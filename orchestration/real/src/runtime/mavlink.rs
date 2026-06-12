use std::io::ErrorKind;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use mavlink::dialects::ardupilotmega as apm;
use mavlink::{MavConnection, MavHeader};
use serde::Serialize;

use crate::tasks::{
    ARDUCOPTER_GUIDED_MODE_ID, CommandAckEvaluation, MAV_CMD_COMPONENT_ARM_DISARM,
    MavlinkStatusText, evaluate_command_ack,
};

const DEFAULT_TARGET_SYSTEM: u8 = 1;
const DEFAULT_TARGET_COMPONENT: u8 = 0;
const DEFAULT_SOURCE_SYSTEM: u8 = 255;
const DEFAULT_SOURCE_COMPONENT: u8 = 190;
const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: f32 = 1.0;

#[derive(Debug, Clone)]
pub struct MotorDebugRuntimeRequest {
    pub endpoint: String,
    pub motor_percent: f64,
    pub motor_sec: f64,
    pub target_system: u8,
    pub target_component: u8,
    pub heartbeat_timeout: Duration,
    pub ack_timeout: Duration,
    pub mode_timeout: Duration,
}

#[derive(Debug, Clone, Serialize)]
pub struct MotorDebugRuntimeReport {
    pub ok: bool,
    pub blocked: bool,
    pub blockers: Vec<String>,
    pub connection_endpoint: String,
    pub target_system: u8,
    pub target_component: u8,
    pub initial_heartbeat: Option<MavlinkHeartbeat>,
    pub guided_mode: GuidedModeRuntimeReport,
    pub arm_ack: Option<CommandAckEvaluation>,
    pub disarm_ack: Option<CommandAckEvaluation>,
    pub shutdown_claim: String,
    pub throttle_command_claim: String,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct MavlinkHeartbeat {
    pub system_id: u8,
    pub component_id: u8,
    pub custom_mode: u32,
    pub base_mode_bits: u8,
}

#[derive(Debug, Clone, Serialize)]
pub struct GuidedModeRuntimeReport {
    pub ok: bool,
    pub mode_id: u32,
    pub command_ack: Option<CommandAckEvaluation>,
    pub observed_heartbeat: Option<MavlinkHeartbeat>,
    pub blockers: Vec<String>,
}

pub trait MotorDebugMavlink {
    fn send_set_guided(&mut self, target_system: u8, target_component: u8) -> Result<()>;
    fn send_arm_disarm(&mut self, target_system: u8, target_component: u8, arm: bool)
    -> Result<()>;
    fn next_event(&mut self, timeout: Duration) -> Result<Option<MavlinkEvent>>;
}

#[derive(Debug, Clone, PartialEq)]
pub enum MavlinkEvent {
    Heartbeat(MavlinkHeartbeat),
    CommandAck(CommandAckEvaluation),
    StatusText(MavlinkStatusText),
}

pub struct ArdupilotMegaConnection {
    connection: mavlink::Connection<apm::MavMessage>,
}

impl ArdupilotMegaConnection {
    pub fn connect(endpoint: &str) -> Result<Self> {
        let connection = mavlink::connect::<apm::MavMessage>(endpoint)
            .with_context(|| format!("connect MAVLink endpoint {endpoint}"))?;
        Ok(Self { connection })
    }
}

impl MotorDebugMavlink for ArdupilotMegaConnection {
    fn send_set_guided(&mut self, target_system: u8, target_component: u8) -> Result<()> {
        let message = apm::MavMessage::COMMAND_LONG(apm::COMMAND_LONG_DATA {
            target_system,
            target_component,
            command: apm::MavCmd::MAV_CMD_DO_SET_MODE,
            confirmation: 0,
            param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2: ARDUCOPTER_GUIDED_MODE_ID as f32,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });
        self.connection
            .send(&source_header(), &message)
            .context("send MAV_CMD_DO_SET_MODE GUIDED")?;
        Ok(())
    }

    fn send_arm_disarm(
        &mut self,
        target_system: u8,
        target_component: u8,
        arm: bool,
    ) -> Result<()> {
        let message = apm::MavMessage::COMMAND_LONG(apm::COMMAND_LONG_DATA {
            target_system,
            target_component,
            command: apm::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1: if arm { 1.0 } else { 0.0 },
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });
        self.connection
            .send(&source_header(), &message)
            .context("send MAV_CMD_COMPONENT_ARM_DISARM")?;
        Ok(())
    }

    fn next_event(&mut self, timeout: Duration) -> Result<Option<MavlinkEvent>> {
        let deadline = Instant::now() + timeout;
        loop {
            match self.connection.try_recv() {
                Ok((header, message)) => return Ok(event_from_message(header, message)),
                Err(mavlink::error::MessageReadError::Io(error))
                    if matches!(error.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) =>
                {
                    if Instant::now() >= deadline {
                        return Ok(None);
                    }
                    thread::sleep(Duration::from_millis(20));
                }
                Err(error) => return Err(error).context("receive MAVLink message"),
            }
        }
    }
}

pub fn run_motor_debug_runtime(
    connection: &mut dyn MotorDebugMavlink,
    request: &MotorDebugRuntimeRequest,
) -> Result<MotorDebugRuntimeReport> {
    let mut blockers = Vec::new();
    let initial_heartbeat = wait_for_heartbeat(connection, request.heartbeat_timeout)?;
    if initial_heartbeat.is_none() {
        blockers.push("motor_debug_heartbeat_timeout".to_string());
    }

    let guided_mode = if blockers.is_empty() {
        set_guided_and_wait(connection, request)?
    } else {
        GuidedModeRuntimeReport {
            ok: false,
            mode_id: ARDUCOPTER_GUIDED_MODE_ID,
            command_ack: None,
            observed_heartbeat: None,
            blockers: vec!["motor_debug_guided_mode_not_attempted_without_heartbeat".to_string()],
        }
    };
    blockers.extend(guided_mode.blockers.clone());

    let mut arm_ack = None;
    let mut disarm_ack = None;
    let mut shutdown_claim = "not_requested".to_string();
    let mut throttle_command_claim = "not_sent".to_string();

    if blockers.is_empty() {
        connection.send_arm_disarm(request.target_system, request.target_component, true)?;
        arm_ack = wait_for_command_ack(
            connection,
            MAV_CMD_COMPONENT_ARM_DISARM,
            request.ack_timeout,
        )?;
        if let Some(ack) = &arm_ack {
            blockers.extend(crate::tasks::command_rejection_blockers(
                "motor_debug_arm_rejected",
                ack,
            ));
        } else {
            blockers.push("motor_debug_arm_ack_timeout".to_string());
        }
    }

    if blockers.is_empty() {
        throttle_command_claim = "not_sent_armed_idle_only".to_string();
        thread::sleep(Duration::from_secs_f64(request.motor_sec));
    }

    if arm_ack.as_ref().is_some_and(|ack| ack.accepted) {
        connection.send_arm_disarm(request.target_system, request.target_component, false)?;
        disarm_ack = wait_for_command_ack(
            connection,
            MAV_CMD_COMPONENT_ARM_DISARM,
            request.ack_timeout,
        )?;
        shutdown_claim = match &disarm_ack {
            Some(ack) if ack.accepted => "disarm_accepted".to_string(),
            Some(ack) => {
                blockers.extend(crate::tasks::command_rejection_blockers(
                    "motor_debug_disarm_rejected",
                    ack,
                ));
                "disarm_rejected".to_string()
            }
            None => {
                blockers.push("motor_debug_disarm_ack_timeout".to_string());
                "disarm_ack_timeout".to_string()
            }
        };
    }

    blockers = dedupe(blockers);
    Ok(MotorDebugRuntimeReport {
        ok: blockers.is_empty(),
        blocked: !blockers.is_empty(),
        blockers,
        connection_endpoint: request.endpoint.clone(),
        target_system: request.target_system,
        target_component: request.target_component,
        initial_heartbeat,
        guided_mode,
        arm_ack,
        disarm_ack,
        shutdown_claim,
        throttle_command_claim,
    })
}

pub fn real_motor_debug_runtime(
    request: &MotorDebugRuntimeRequest,
) -> Result<MotorDebugRuntimeReport> {
    let mut connection = ArdupilotMegaConnection::connect(&request.endpoint)?;
    run_motor_debug_runtime(&mut connection, request)
}

impl MotorDebugRuntimeRequest {
    pub fn new(endpoint: String, motor_percent: f64, motor_sec: f64) -> Self {
        Self {
            endpoint,
            motor_percent,
            motor_sec,
            target_system: DEFAULT_TARGET_SYSTEM,
            target_component: DEFAULT_TARGET_COMPONENT,
            heartbeat_timeout: Duration::from_secs(10),
            ack_timeout: Duration::from_secs(5),
            mode_timeout: Duration::from_secs(5),
        }
    }
}

fn set_guided_and_wait(
    connection: &mut dyn MotorDebugMavlink,
    request: &MotorDebugRuntimeRequest,
) -> Result<GuidedModeRuntimeReport> {
    connection.send_set_guided(request.target_system, request.target_component)?;
    let command_ack = wait_for_command_ack(
        connection,
        apm::MavCmd::MAV_CMD_DO_SET_MODE as u32,
        request.ack_timeout,
    )?;
    let observed_heartbeat = wait_for_guided_heartbeat(connection, request.mode_timeout)?;
    let mut blockers = Vec::new();

    match &command_ack {
        Some(ack) => blockers.extend(crate::tasks::command_rejection_blockers(
            "motor_debug_guided_mode_rejected",
            ack,
        )),
        None => blockers.push("motor_debug_guided_mode_ack_timeout".to_string()),
    }
    if observed_heartbeat.is_none() {
        blockers.push("motor_debug_guided_mode_heartbeat_timeout".to_string());
    }

    blockers = dedupe(blockers);
    Ok(GuidedModeRuntimeReport {
        ok: blockers.is_empty(),
        mode_id: ARDUCOPTER_GUIDED_MODE_ID,
        command_ack,
        observed_heartbeat,
        blockers,
    })
}

fn wait_for_heartbeat(
    connection: &mut dyn MotorDebugMavlink,
    timeout: Duration,
) -> Result<Option<MavlinkHeartbeat>> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Ok(None);
        }
        if let Some(MavlinkEvent::Heartbeat(heartbeat)) = connection.next_event(remaining)? {
            return Ok(Some(heartbeat));
        }
    }
}

fn wait_for_guided_heartbeat(
    connection: &mut dyn MotorDebugMavlink,
    timeout: Duration,
) -> Result<Option<MavlinkHeartbeat>> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Ok(None);
        }
        if let Some(MavlinkEvent::Heartbeat(heartbeat)) = connection.next_event(remaining)?
            && heartbeat.custom_mode == ARDUCOPTER_GUIDED_MODE_ID
        {
            return Ok(Some(heartbeat));
        }
    }
}

fn wait_for_command_ack(
    connection: &mut dyn MotorDebugMavlink,
    command: u32,
    timeout: Duration,
) -> Result<Option<CommandAckEvaluation>> {
    let deadline = Instant::now() + timeout;
    let mut status_text = Vec::new();
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Ok(None);
        }
        match connection.next_event(remaining)? {
            Some(MavlinkEvent::CommandAck(ack)) if ack.command == command => {
                return Ok(Some(CommandAckEvaluation { status_text, ..ack }));
            }
            Some(MavlinkEvent::StatusText(status)) => status_text.push(status),
            Some(_) | None => {}
        }
    }
}

fn event_from_message(header: MavHeader, message: apm::MavMessage) -> Option<MavlinkEvent> {
    match message {
        apm::MavMessage::HEARTBEAT(data) => Some(MavlinkEvent::Heartbeat(MavlinkHeartbeat {
            system_id: header.system_id,
            component_id: header.component_id,
            custom_mode: data.custom_mode,
            base_mode_bits: data.base_mode.bits(),
        })),
        apm::MavMessage::COMMAND_ACK(data) => Some(MavlinkEvent::CommandAck(evaluate_command_ack(
            data.command as u32,
            data.result as u32,
            0,
            Vec::new(),
        ))),
        apm::MavMessage::STATUSTEXT(data) => Some(MavlinkEvent::StatusText(MavlinkStatusText {
            severity: data.severity as u8,
            text: data.text.to_str().unwrap_or("").to_string(),
        })),
        _ => None,
    }
}

fn source_header() -> MavHeader {
    MavHeader {
        system_id: DEFAULT_SOURCE_SYSTEM,
        component_id: DEFAULT_SOURCE_COMPONENT,
        sequence: 0,
    }
}

fn dedupe(values: Vec<String>) -> Vec<String> {
    let mut deduped = Vec::new();
    for value in values {
        if !deduped.contains(&value) {
            deduped.push(value);
        }
    }
    deduped
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[derive(Debug, Default)]
    struct FakeMavlink {
        events: VecDeque<MavlinkEvent>,
        sent_guided: usize,
        sent_arm: usize,
        sent_disarm: usize,
    }

    impl MotorDebugMavlink for FakeMavlink {
        fn send_set_guided(&mut self, _target_system: u8, _target_component: u8) -> Result<()> {
            self.sent_guided += 1;
            Ok(())
        }

        fn send_arm_disarm(
            &mut self,
            _target_system: u8,
            _target_component: u8,
            arm: bool,
        ) -> Result<()> {
            if arm {
                self.sent_arm += 1;
            } else {
                self.sent_disarm += 1;
            }
            Ok(())
        }

        fn next_event(&mut self, _timeout: Duration) -> Result<Option<MavlinkEvent>> {
            Ok(self.events.pop_front())
        }
    }

    fn request() -> MotorDebugRuntimeRequest {
        MotorDebugRuntimeRequest {
            endpoint: "udpin:127.0.0.1:14550".to_string(),
            motor_percent: 5.0,
            motor_sec: 0.0,
            target_system: 1,
            target_component: 0,
            heartbeat_timeout: Duration::from_millis(1),
            ack_timeout: Duration::from_millis(1),
            mode_timeout: Duration::from_millis(1),
        }
    }

    fn heartbeat(custom_mode: u32) -> MavlinkEvent {
        MavlinkEvent::Heartbeat(MavlinkHeartbeat {
            system_id: 1,
            component_id: 1,
            custom_mode,
            base_mode_bits: 1,
        })
    }

    fn accepted(command: u32) -> MavlinkEvent {
        MavlinkEvent::CommandAck(evaluate_command_ack(command, 0, 0, Vec::new()))
    }

    #[test]
    fn motor_debug_runtime_sends_guided_arm_and_disarm() {
        let mut mavlink = FakeMavlink {
            events: VecDeque::from([
                heartbeat(0),
                accepted(apm::MavCmd::MAV_CMD_DO_SET_MODE as u32),
                heartbeat(ARDUCOPTER_GUIDED_MODE_ID),
                accepted(MAV_CMD_COMPONENT_ARM_DISARM),
                accepted(MAV_CMD_COMPONENT_ARM_DISARM),
            ]),
            ..FakeMavlink::default()
        };

        let report = run_motor_debug_runtime(&mut mavlink, &request()).expect("runtime");

        assert!(report.ok);
        assert_eq!(mavlink.sent_guided, 1);
        assert_eq!(mavlink.sent_arm, 1);
        assert_eq!(mavlink.sent_disarm, 1);
        assert_eq!(report.shutdown_claim, "disarm_accepted");
        assert_eq!(report.throttle_command_claim, "not_sent_armed_idle_only");
    }

    #[test]
    fn motor_debug_runtime_blocks_arm_rejection_and_disarms_not_sent() {
        let mut mavlink = FakeMavlink {
            events: VecDeque::from([
                heartbeat(0),
                accepted(apm::MavCmd::MAV_CMD_DO_SET_MODE as u32),
                heartbeat(ARDUCOPTER_GUIDED_MODE_ID),
                MavlinkEvent::StatusText(MavlinkStatusText {
                    severity: 3,
                    text: "PreArm: safety switch".to_string(),
                }),
                MavlinkEvent::CommandAck(evaluate_command_ack(
                    MAV_CMD_COMPONENT_ARM_DISARM,
                    4,
                    0,
                    Vec::new(),
                )),
            ]),
            ..FakeMavlink::default()
        };

        let report = run_motor_debug_runtime(&mut mavlink, &request()).expect("runtime");

        assert!(!report.ok);
        assert_eq!(mavlink.sent_arm, 1);
        assert_eq!(mavlink.sent_disarm, 0);
        assert_eq!(
            report.blockers,
            vec![
                "motor_debug_arm_rejected:MAV_RESULT_FAILED",
                "motor_debug_arm_rejected_status:PreArm: safety switch"
            ]
        );
    }

    #[test]
    fn motor_debug_runtime_blocks_without_heartbeat() {
        let mut mavlink = FakeMavlink::default();

        let report = run_motor_debug_runtime(&mut mavlink, &request()).expect("runtime");

        assert!(!report.ok);
        assert_eq!(
            report.blockers,
            vec![
                "motor_debug_heartbeat_timeout",
                "motor_debug_guided_mode_not_attempted_without_heartbeat"
            ]
        );
        assert_eq!(mavlink.sent_guided, 0);
        assert_eq!(mavlink.sent_arm, 0);
    }
}
