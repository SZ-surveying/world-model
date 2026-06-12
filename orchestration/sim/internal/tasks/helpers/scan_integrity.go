package helpers

import (
	"sort"
	"strings"
)

type MotorOutputSummary struct {
	MotorOutputClaim        string   `json:"motor_output_claim"`
	Available               bool     `json:"available"`
	CandidateTopics         []string `json:"candidate_topics"`
	MotorPWMMin             *float64 `json:"motor_pwm_min"`
	MotorPWMMax             *float64 `json:"motor_pwm_max"`
	MotorPWMSpread          *float64 `json:"motor_pwm_spread"`
	MotorRPMMin             *float64 `json:"motor_rpm_min"`
	MotorRPMMax             *float64 `json:"motor_rpm_max"`
	MotorRPMSpread          *float64 `json:"motor_rpm_spread"`
	MotorThrustBiasEstimate *float64 `json:"motor_thrust_bias_estimate"`
	Reason                  string   `json:"reason"`
}

func MotorOutputSummaryFromTopics(topics []string) MotorOutputSummary {
	keywords := []string{"motor", "servo", "actuator", "esc", "rpm", "pwm"}
	excluded := map[string]bool{"/robot_description": true}
	var candidates []string
	for _, topic := range topics {
		lower := strings.ToLower(topic)
		if excluded[topic] || strings.Contains(lower, "support_motor") {
			continue
		}
		for _, keyword := range keywords {
			if strings.Contains(lower, keyword) {
				candidates = append(candidates, topic)
				break
			}
		}
	}
	sort.Strings(candidates)
	if len(candidates) == 0 {
		return MotorOutputSummary{
			MotorOutputClaim: "not_available",
			Available:        false,
			CandidateTopics:  []string{},
			Reason:           "no motor/servo/actuator/ESC output topic is exposed in the ROS graph",
		}
	}
	return MotorOutputSummary{
		MotorOutputClaim: "candidate_topics_present",
		Available:        false,
		CandidateTopics:  candidates,
		Reason:           "candidate topics exist, but P10.1 does not parse motor output message schemas yet",
	}
}
