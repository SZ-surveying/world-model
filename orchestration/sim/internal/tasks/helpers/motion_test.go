package helpers

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestBuildMotionDoctorSummaryPasses(t *testing.T) {
	profile := filepath.Join(t.TempDir(), "motion_topics.txt")
	if err := os.WriteFile(profile, []byte("required /slam/odom\noptional /tf\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	spec := DefaultMotionGateSpec()
	spec.RosbagProfile = profile

	summary := BuildMotionDoctorSummary(spec, DependencyDoctor{OK: true, Blockers: []string{}}, true)
	if !summary.OK || summary.Blocked {
		t.Fatalf("summary = %#v", summary)
	}
	if len(summary.MotionGateDoctor.RosbagProfile.RequiredTopics) != 1 {
		t.Fatalf("rosbag profile = %#v", summary.MotionGateDoctor.RosbagProfile)
	}
}

func TestBuildMotionDoctorSummaryFindsContractBlockers(t *testing.T) {
	spec := DefaultMotionGateSpec()
	spec.RosbagProfile = filepath.Join(t.TempDir(), "missing.txt")
	spec.UsesGazeboTruthAsInput = true
	spec.SlamOdomTopic = "/navlab/truth/odom"
	spec.CmdVelTopic = "/wrong/cmd_vel"

	summary := BuildMotionDoctorSummary(spec, DependencyDoctor{OK: false, Blockers: []string{"hover_failed"}}, true)
	for _, expected := range []string{
		"hover_failed",
		"motion rosbag profile is missing or empty",
		"motion gate must not use Gazebo truth as a control/planning/SLAM/ExternalNav input",
		"motion gate SLAM odom topic must match canonical SLAM odom topic",
		"motion gate SLAM odom topic must not be the Gazebo truth diagnostic topic",
		"motion gate cmd_vel topic must match the FCU controller output topic",
	} {
		if !containsString(summary.Blockers, expected) {
			t.Fatalf("expected blocker %q in %#v", expected, summary.Blockers)
		}
	}
}

func TestMotionFoxgloveNotes(t *testing.T) {
	notes := MotionFoxgloveNotes(DefaultMotionGateSpec())
	if !strings.Contains(notes, "Motion status") || !strings.Contains(notes, "/navlab/motion/status") {
		t.Fatalf("notes missing motion topic:\n%s", notes)
	}
}

func TestMotionHelperMarkedPortedBasic(t *testing.T) {
	definition, err := DefaultRegistry().Get("motion")
	if err != nil {
		t.Fatal(err)
	}
	if definition.MigrationStatus != "ported_basic" {
		t.Fatalf("motion status = %q, want ported_basic", definition.MigrationStatus)
	}
}

func containsString(values []string, expected string) bool {
	for _, value := range values {
		if value == expected {
			return true
		}
	}
	return false
}
