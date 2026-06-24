package tasks

import (
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"github.com/foxglove/mcap/go/mcap"
	"github.com/klauspost/compress/zstd"
)

type hoverContractSpec struct {
	Key              string
	Topic            string
	MessageType      string
	Scope            string
	Role             string
	ExpectedFrame    string
	ExpectedChild    string
	Origin           string
	Units            string
	TimestampSource  string
	Semantics        string
	ComparisonRule   string
	RuntimeInfluence string
	RealityRelevance string
}

type observedContractTopic struct {
	SampleCount       int
	FirstLogTimeSec   float64
	LastLogTimeSec    float64
	FirstHeaderSec    *float64
	LastHeaderSec     *float64
	FrameCounts       map[string]int
	ChildFrameCounts  map[string]int
	LatestStatus      map[string]any
	TransformPairRows map[string]int
}

var phase43ContractSpecs = []hoverContractSpec{
	{
		Key:              "slam_odom",
		Topic:            "/slam/odom",
		MessageType:      "nav_msgs/msg/Odometry",
		Scope:            "real-flight-safety",
		Role:             "primary SLAM estimate candidate for direct ExternalNav route",
		ExpectedFrame:    "map",
		ExpectedChild:    "base_link",
		Origin:           "SLAM map origin set by Cartographer/localization initialization",
		Units:            "meters, radians",
		TimestampSource:  "ROS message header stamp from SLAM adapter / sim time",
		Semantics:        "ROS world/map estimate with body child frame",
		ComparisonRule:   "Compare only as relative motion after map/frame contract is known; not a global truth source.",
		RuntimeInfluence: "Can feed ExternalNav bridge in slam-direct profiles.",
		RealityRelevance: "Real UAV ExternalNav can use this class of SLAM odometry if stable and frame-explicit.",
	},
	{
		Key:              "external_nav_odom",
		Topic:            "/external_nav/odom",
		MessageType:      "nav_msgs/msg/Odometry",
		Scope:            "real-flight-safety",
		Role:             "ROS odometry sent onward as MAVLink ODOMETRY input",
		ExpectedFrame:    "external_nav",
		ExpectedChild:    "base_link",
		Origin:           "ExternalNav bridge output origin; derived from input odometry with height merge",
		Units:            "meters, radians",
		TimestampSource:  "ROS message header stamp from external_nav bridge",
		Semantics:        "ROS ENU/FLU-style odometry before MAVLink conversion",
		ComparisonRule:   "Compare to /slam/odom only through bridge contract; compare to FCU only through MAVLink ENU/NED mapping.",
		RuntimeInfluence: "Input to MAVLink ExternalNav sender.",
		RealityRelevance: "Directly represents the real-flight ExternalNav estimate handed to FCU/EKF.",
	},
	{
		Key:              "external_nav_odom_candidate",
		Topic:            "/external_nav/odom_candidate",
		MessageType:      "nav_msgs/msg/Odometry",
		Scope:            "legacy scan-reference route",
		Role:             "legacy selector candidate before ExternalNav bridge",
		ExpectedFrame:    "map",
		ExpectedChild:    "base_link",
		Origin:           "selector output origin; legacy scan-reference anchoring may apply",
		Units:            "meters, radians",
		TimestampSource:  "ROS message header stamp from selector",
		Semantics:        "candidate odometry, not current direct-route primary source",
		ComparisonRule:   "Legacy/experimental; do not use to justify current direct /slam/odom route unless Phase 43 revalidates it.",
		RuntimeInfluence: "Feeds ExternalNav bridge only in selector profiles.",
		RealityRelevance: "Not current primary real-flight candidate.",
	},
	{
		Key:              "fcu_local_position_pose",
		Topic:            "/navlab/fcu/local_position_pose",
		MessageType:      "geometry_msgs/msg/PoseStamped",
		Scope:            "real-flight-safety",
		Role:             "FCU LOCAL_POSITION_NED mirrored into ROS pose",
		ExpectedFrame:    "map",
		ExpectedChild:    "",
		Origin:           "FCU EKF local origin, converted to ROS pose by ExternalNav sender",
		Units:            "meters, radians",
		TimestampSource:  "ROS receive time of FCU LOCAL_POSITION_NED mirror",
		Semantics:        "FCU EKF local estimate converted from NED to ROS pose",
		ComparisonRule:   "Compare to ExternalNav only through documented MAVLink/FCU local-frame mapping and time alignment.",
		RuntimeInfluence: "Status/validation source; not the ExternalNav input itself.",
		RealityRelevance: "Real UAV must expose this or equivalent EKF feedback for acceptance/loss validation.",
	},
	{
		Key:              "mavlink_odometry_status",
		Topic:            "/mavlink_external_nav/status",
		MessageType:      "std_msgs/msg/String JSON",
		Scope:            "real-flight-safety",
		Role:             "MAVLink ODOMETRY sender contract/status",
		ExpectedFrame:    "MAV_FRAME_LOCAL_FRD",
		ExpectedChild:    "MAV_FRAME_BODY_FRD",
		Origin:           "FCU local frame via ArduPilot ExternalNav interface",
		Units:            "meters, radians, microseconds",
		TimestampSource:  "sender monotonic clock for MAVLink time_usec; ROS receive time for status",
		Semantics:        "ExternalNav estimate injected into ArduPilot EKF",
		ComparisonRule:   "Validate field map and freshness; do not compare raw fields without ENU/NED conversion.",
		RuntimeInfluence: "Sends MAVLink ODOMETRY to FCU.",
		RealityRelevance: "Primary real-flight FCU/EKF contract.",
	},
	{
		Key:              "gazebo_model_odometry",
		Topic:            "/gazebo/model/odometry",
		MessageType:      "nav_msgs/msg/Odometry",
		Scope:            "sim-review-only",
		Role:             "Gazebo model odometry review source",
		ExpectedFrame:    "odom",
		ExpectedChild:    "base_link",
		Origin:           "Gazebo model odometry publisher origin/model contract",
		Units:            "meters, radians",
		TimestampSource:  "Gazebo/sim time through bridge",
		Semantics:        "simulation review source, not runtime ExternalNav input",
		ComparisonRule:   "Review-only until model/link/canonical-link/frame/time contract is proven.",
		RuntimeInfluence: "Must not feed runtime ExternalNav or mission decisions directly.",
		RealityRelevance: "Useful to reduce real-flight risk only after review-source contract is audited.",
	},
	{
		Key:              "gazebo_tf",
		Topic:            "/gazebo/tf",
		MessageType:      "tf2_msgs/msg/TFMessage",
		Scope:            "sim-review-only",
		Role:             "Gazebo dynamic pose bridge source used by review odometry",
		ExpectedFrame:    "",
		ExpectedChild:    "",
		Origin:           "Gazebo world dynamic pose stream",
		Units:            "meters, radians",
		TimestampSource:  "Gazebo/sim time through bridge",
		Semantics:        "simulation world/model/link transform stream",
		ComparisonRule:   "Only identifies Gazebo model/link source; not directly comparable to SLAM without selected-transform contract.",
		RuntimeInfluence: "Review-only.",
		RealityRelevance: "No direct real-flight equivalent; analogous to external motion-capture review source.",
	},
	{
		Key:              "gazebo_tf_static",
		Topic:            "/gazebo/tf_static",
		MessageType:      "tf2_msgs/msg/TFMessage",
		Scope:            "sim-review-only",
		Role:             "Gazebo static transform review source",
		ExpectedFrame:    "",
		ExpectedChild:    "",
		Origin:           "Gazebo/static model frame stream",
		Units:            "meters, radians",
		TimestampSource:  "Gazebo/sim time through bridge",
		Semantics:        "simulation static transform stream",
		ComparisonRule:   "Review-only; use to validate model/link frame contract.",
		RuntimeInfluence: "Review-only.",
		RealityRelevance: "No direct real-flight equivalent except calibrated sensor extrinsics.",
	},
	{
		Key:              "scan_reference_drift_odom",
		Topic:            "/navlab/scan_reference_drift/odom",
		MessageType:      "nav_msgs/msg/Odometry",
		Scope:            "legacy scan-reference route",
		Role:             "scan-derived diagnostic/legacy measurement",
		ExpectedFrame:    "scan_reference",
		ExpectedChild:    "base_link",
		Origin:           "scan reference frame captured by scan-reference node",
		Units:            "meters, radians",
		TimestampSource:  "scan-reference node message stamp",
		Semantics:        "measurement/correction signal, not current primary odometry source",
		ComparisonRule:   "Do not compare as map pose unless transform/anchor/intent contract is explicitly active.",
		RuntimeInfluence: "Legacy selector/prior route only.",
		RealityRelevance: "Experimental; not needed if direct /slam/odom route is accepted.",
	},
	{
		Key:              "cartographer_odometry_input",
		Topic:            "/cartographer/odometry_input",
		MessageType:      "nav_msgs/msg/Odometry",
		Scope:            "legacy scan-reference route",
		Role:             "Cartographer odometry prior input",
		ExpectedFrame:    "odom",
		ExpectedChild:    "base_link",
		Origin:           "scan-reference generated odometry prior when use_odometry=true",
		Units:            "meters, radians",
		TimestampSource:  "prior node message stamp",
		Semantics:        "diagnostic in no-odom-prior route; runtime prior only when Cartographer use_odometry=true",
		ComparisonRule:   "In no-odom-prior profiles it must not influence /slam/odom.",
		RuntimeInfluence: "Should be ignored when Cartographer use_odometry=false.",
		RealityRelevance: "Legacy/experimental unless a real sensor odometry prior is explicitly validated.",
	},
}

// BuildHoverContractAudit builds Phase 43 topic-contract evidence from one hover artifact.
// It is diagnostic-only and must not change runtime control behavior.
func BuildHoverContractAudit(artifactDir string) (map[string]any, error) {
	path := filepath.Join(artifactDir, "rosbag", "hover_rosbag", "hover_rosbag_0.mcap.zstd")
	if _, err := os.Stat(path); err != nil {
		path = filepath.Join(artifactDir, "rosbag", "hover_rosbag", "hover_rosbag_0.mcap")
	}
	return summarizeHoverContractAudit(path, artifactDir)
}

func summarizeHoverContractAudit(path string, artifactDir string) (map[string]any, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer func() { _ = file.Close() }()
	var stream io.Reader = file
	var decoder *zstd.Decoder
	if filepath.Ext(path) == ".zstd" {
		decoder, err = zstd.NewReader(file)
		if err != nil {
			return nil, err
		}
		defer decoder.Close()
		stream = decoder
	}
	reader, err := mcap.NewReader(stream)
	if err != nil {
		return nil, err
	}
	defer reader.Close()
	it, err := reader.Messages(mcap.UsingIndex(false))
	if err != nil {
		return nil, err
	}

	specByTopic := map[string]hoverContractSpec{}
	observed := map[string]*observedContractTopic{}
	for _, spec := range phase43ContractSpecs {
		specByTopic[spec.Topic] = spec
		observed[spec.Key] = &observedContractTopic{
			FrameCounts:       map[string]int{},
			ChildFrameCounts:  map[string]int{},
			TransformPairRows: map[string]int{},
		}
	}

	bagStartSec := 0.0
	bagEndSec := 0.0
	for {
		_, channel, message, err := it.Next(nil) //nolint:staticcheck
		if errorsIsEOF(err) {
			break
		}
		if err != nil {
			return nil, err
		}
		spec, ok := specByTopic[channel.Topic]
		if !ok {
			continue
		}
		logTimeSec := float64(message.LogTime) / 1e9
		if bagStartSec == 0 || logTimeSec < bagStartSec {
			bagStartSec = logTimeSec
		}
		if logTimeSec > bagEndSec {
			bagEndSec = logTimeSec
		}
		row := observed[spec.Key]
		row.SampleCount++
		if row.FirstLogTimeSec == 0 {
			row.FirstLogTimeSec = logTimeSec
		}
		row.LastLogTimeSec = logTimeSec
		switch spec.MessageType {
		case "nav_msgs/msg/Odometry":
			sample, err := parseOdometryPoseCDR(message.Data)
			if err != nil {
				return nil, fmt.Errorf("parse %s: %w", channel.Topic, err)
			}
			incrementStringCount(row.FrameCounts, sample.FrameID)
			incrementStringCount(row.ChildFrameCounts, sample.ChildFrameID)
			if headerSec, err := parseHeaderStampSecCDR(message.Data); err == nil {
				setHeaderRange(row, headerSec)
			}
		case "geometry_msgs/msg/PoseStamped":
			sample, err := parsePoseStampedPoseCDR(message.Data)
			if err != nil {
				return nil, fmt.Errorf("parse %s: %w", channel.Topic, err)
			}
			incrementStringCount(row.FrameCounts, sample.FrameID)
			if headerSec, err := parseHeaderStampSecCDR(message.Data); err == nil {
				setHeaderRange(row, headerSec)
			}
		case "std_msgs/msg/String JSON":
			payload, err := parseJSONStatusCDR(message.Data)
			if err != nil {
				return nil, fmt.Errorf("parse %s: %w", channel.Topic, err)
			}
			row.LatestStatus = payload
			if frame, _ := payload["frame_id"].(string); frame != "" {
				incrementStringCount(row.FrameCounts, frame)
			}
			if child, _ := payload["child_frame_id"].(string); child != "" {
				incrementStringCount(row.ChildFrameCounts, child)
			}
			if mavFrame, _ := payload["mav_frame_id"].(string); mavFrame != "" {
				incrementStringCount(row.FrameCounts, mavFrame)
			}
			if mavChild, _ := payload["mav_child_frame_id"].(string); mavChild != "" {
				incrementStringCount(row.ChildFrameCounts, mavChild)
			}
		case "tf2_msgs/msg/TFMessage":
			pairs, err := parseTFMessagePairsCDR(message.Data)
			if err != nil {
				return nil, fmt.Errorf("parse %s: %w", channel.Topic, err)
			}
			for _, pair := range pairs {
				incrementStringCount(row.FrameCounts, pair.FrameID)
				incrementStringCount(row.ChildFrameCounts, pair.ChildFrameID)
				incrementStringCount(row.TransformPairRows, pair.FrameID+"->"+pair.ChildFrameID)
				if pair.HeaderStampSec != nil {
					setHeaderRange(row, *pair.HeaderStampSec)
				}
			}
		}
	}

	topics := map[string]any{}
	blockers := []string{}
	for _, spec := range phase43ContractSpecs {
		row := observed[spec.Key]
		summary := contractTopicSummary(spec, row, bagStartSec)
		topics[spec.Key] = summary
		if row.SampleCount == 0 {
			if spec.Topic == "/gazebo/tf_static" {
				blockers = append(blockers, "gazebo_tf_static_missing_review_source")
			} else {
				blockers = append(blockers, "missing:"+spec.Topic)
			}
			continue
		}
		if spec.ExpectedFrame != "" && !stringCountContains(row.FrameCounts, spec.ExpectedFrame) {
			blockers = append(blockers, "frame_contract_unproven:"+spec.Topic)
		}
		if spec.ExpectedChild != "" && !stringCountContains(row.ChildFrameCounts, spec.ExpectedChild) {
			blockers = append(blockers, "child_frame_contract_unproven:"+spec.Topic)
		}
	}
	bridgeEvidence := parseGazeboBridgeOdometryEvidence(filepath.Join(artifactDir, "bridge_override.yaml"))
	modelEvidence := parseGazeboModelOverlayOdometryEvidence(filepath.Join(artifactDir, "model_overlay.sdf"))
	blockers = append(blockers, gazeboContractBlockers(bridgeEvidence, modelEvidence)...)

	return map[string]any{
		"schema":                    "navlab.hover_contract_audit.v1",
		"diagnostic_only":           true,
		"runtime_control_unchanged": true,
		"phase":                     "Phase 43",
		"rosbag_path":               path,
		"artifact_dir":              artifactDir,
		"bag": map[string]any{
			"start_sec":    bagStartSec,
			"end_sec":      bagEndSec,
			"duration_sec": math.Max(0, bagEndSec-bagStartSec),
		},
		"topics":                 topics,
		"blockers":               blockers,
		"gazebo_bridge_evidence": bridgeEvidence,
		"gazebo_model_evidence":  modelEvidence,
		"decision_rule":          "No source is treated as truth by this audit. Compare sources only through documented frame/origin/time transforms.",
	}, nil
}

func gazeboContractBlockers(bridgeEvidence map[string]any, modelEvidence map[string]any) []string {
	blockers := []string{}
	if bridgeEvidence == nil || modelEvidence == nil {
		return blockers
	}
	bridgeTopic, _ := bridgeEvidence["bridge_gz_topic_name"].(string)
	expectedTopic, _ := modelEvidence["expected_bridge_gz_topic_name"].(string)
	if strings.Contains(bridgeTopic, "{{") || strings.Contains(bridgeTopic, "}}") {
		blockers = append(blockers, "gazebo_bridge_topic_runtime_substitution_unverified")
	}
	if bridgeTopic != "" && expectedTopic != "" && bridgeTopic != expectedTopic {
		blockers = append(blockers, "gazebo_bridge_topic_model_name_resolution_unverified")
	}
	return blockers
}

type tfPairObservation struct {
	FrameID        string
	ChildFrameID   string
	HeaderStampSec *float64
}

func parseTFMessagePairsCDR(data []byte) ([]tfPairObservation, error) {
	cursor := gateCDRCursor{data: data}
	if err := cursor.skip(4); err != nil {
		return nil, err
	}
	length, err := cursor.uint32()
	if err != nil {
		return nil, err
	}
	pairs := make([]tfPairObservation, 0, int(length))
	for idx := 0; idx < int(length); idx++ {
		sec, err := cursor.int32()
		if err != nil {
			return nil, err
		}
		nsec, err := cursor.uint32()
		if err != nil {
			return nil, err
		}
		headerStamp := float64(sec) + (float64(nsec) / 1e9)
		frameID, err := cursor.stringValue()
		if err != nil {
			return nil, err
		}
		childFrameID, err := cursor.stringValue()
		if err != nil {
			return nil, err
		}
		// translation xyz + rotation xyzw
		for field := 0; field < 7; field++ {
			if _, err := cursor.float64(); err != nil {
				return nil, err
			}
		}
		pairs = append(pairs, tfPairObservation{
			FrameID:        frameID,
			ChildFrameID:   childFrameID,
			HeaderStampSec: &headerStamp,
		})
	}
	return pairs, nil
}

func contractTopicSummary(spec hoverContractSpec, row *observedContractTopic, bagStartSec float64) map[string]any {
	summary := map[string]any{
		"topic":             spec.Topic,
		"message_type":      spec.MessageType,
		"scope":             spec.Scope,
		"role":              spec.Role,
		"expected_frame":    spec.ExpectedFrame,
		"expected_child":    spec.ExpectedChild,
		"origin":            spec.Origin,
		"units":             spec.Units,
		"timestamp_source":  spec.TimestampSource,
		"semantics":         spec.Semantics,
		"comparison_rule":   spec.ComparisonRule,
		"runtime_influence": spec.RuntimeInfluence,
		"reality_relevance": spec.RealityRelevance,
		"observed": map[string]any{
			"sample_count": lenIfNil(row),
		},
	}
	if row == nil {
		return summary
	}
	observed := mapFromAny(summary["observed"])
	observed["sample_count"] = row.SampleCount
	observed["first_log_time_sec_from_bag_start"] = relTime(row.FirstLogTimeSec, bagStartSec)
	observed["last_log_time_sec_from_bag_start"] = relTime(row.LastLogTimeSec, bagStartSec)
	observed["frame_counts"] = sortedCountRows(row.FrameCounts)
	observed["child_frame_counts"] = sortedCountRows(row.ChildFrameCounts)
	if row.FirstHeaderSec != nil {
		observed["first_header_stamp_sec"] = *row.FirstHeaderSec
	}
	if row.LastHeaderSec != nil {
		observed["last_header_stamp_sec"] = *row.LastHeaderSec
	}
	if row.LatestStatus != nil {
		observed["latest_status"] = subsetMap(
			row.LatestStatus,
			"state", "ready", "input_topic", "frame_id", "child_frame_id", "mav_frame_id", "mav_child_frame_id",
			"quality", "reset_counter", "time_usec_source", "mapping", "local_position_count", "fcu_local_position_ready",
		)
	}
	if len(row.TransformPairRows) > 0 {
		observed["transform_pairs"] = sortedCountRows(row.TransformPairRows)
	}
	observed["frame_contract_status"] = observedFrameContractStatus(row.FrameCounts, spec.ExpectedFrame)
	observed["child_frame_contract_status"] = observedFrameContractStatus(row.ChildFrameCounts, spec.ExpectedChild)
	return summary
}

func observedFrameContractStatus(counts map[string]int, expected string) string {
	if expected == "" {
		return "not_required"
	}
	if stringCountContains(counts, expected) {
		return "observed_expected"
	}
	if len(counts) == 0 {
		return "missing_observation"
	}
	return "expected_not_observed"
}

func incrementStringCount(counts map[string]int, value string) {
	if value == "" {
		value = "<empty>"
	}
	counts[value]++
}

func stringCountContains(counts map[string]int, value string) bool {
	_, ok := counts[value]
	return ok
}

func setHeaderRange(row *observedContractTopic, headerSec float64) {
	if row.FirstHeaderSec == nil {
		row.FirstHeaderSec = &headerSec
	}
	row.LastHeaderSec = &headerSec
}

func lenIfNil(row *observedContractTopic) int {
	if row == nil {
		return 0
	}
	return row.SampleCount
}

func sortedCountRows(counts map[string]int) []map[string]any {
	keys := make([]string, 0, len(counts))
	for key := range counts {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	rows := make([]map[string]any, 0, len(keys))
	for _, key := range keys {
		rows = append(rows, map[string]any{"value": key, "count": counts[key]})
	}
	return rows
}
