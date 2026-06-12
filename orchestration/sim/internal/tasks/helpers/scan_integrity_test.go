package helpers

import "testing"

func TestMotorOutputSummaryFromTopicsNoCandidates(t *testing.T) {
	summary := MotorOutputSummaryFromTopics([]string{"/scan", "/robot_description", "/support_motor/debug"})
	if summary.MotorOutputClaim != "not_available" {
		t.Fatalf("claim = %q, want not_available", summary.MotorOutputClaim)
	}
	if len(summary.CandidateTopics) != 0 {
		t.Fatalf("candidates = %#v, want empty", summary.CandidateTopics)
	}
}

func TestMotorOutputSummaryFromTopicsCandidates(t *testing.T) {
	summary := MotorOutputSummaryFromTopics([]string{"/scan", "/esc/status", "/motor/pwm"})
	if summary.MotorOutputClaim != "candidate_topics_present" {
		t.Fatalf("claim = %q, want candidate_topics_present", summary.MotorOutputClaim)
	}
	if len(summary.CandidateTopics) != 2 {
		t.Fatalf("candidates = %#v, want 2", summary.CandidateTopics)
	}
}
