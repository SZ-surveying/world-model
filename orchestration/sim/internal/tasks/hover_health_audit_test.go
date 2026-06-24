package tasks

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
)

func TestHoverMetricSpecClassifyUsesTargetHardCapAndReviewOnly(t *testing.T) {
	spec := HoverMetricSpec{Key: "pair.slam_vs_external_nav.p95_error_m", Unit: "m", Tier: HoverTierARealFlightSafety, TargetMax: 0.10, HardMax: 0.15}
	if got := spec.Classify(0.08); got.Band != HoverHealthGreen || got.Severity != HoverMetricInfo {
		t.Fatalf("green classify = %#v", got)
	}
	if got := spec.Classify(0.12); got.Band != HoverHealthYellow || got.Severity != HoverMetricWarning {
		t.Fatalf("yellow classify = %#v", got)
	}
	if got := spec.Classify(0.16); got.Band != HoverHealthRed || got.Severity != HoverMetricHard {
		t.Fatalf("red classify = %#v", got)
	}

	reviewSpec := spec
	reviewSpec.Key = "pair.gazebo_vs_slam.p95_error_m"
	reviewSpec.Tier = HoverTierBReviewOnly
	reviewSpec.ReviewOnly = true
	if got := reviewSpec.Classify(0.16); got.Band != HoverHealthYellow || got.Severity != HoverMetricWarning || !got.ReviewOnly {
		t.Fatalf("review-only classify = %#v", got)
	}
}

func TestBuildHoverHealthAuditClassifiesGazeboOnlyBlockerAsYellow(t *testing.T) {
	dir := writeHoverHealthArtifact(t, []map[string]any{{"code": "hover_gazebo_model_horizontal_drift", "message": "review drift high"}})
	audit, err := BuildHoverHealthAudit(dir)
	if err != nil {
		t.Fatal(err)
	}
	if audit.HealthBand != HoverHealthYellow {
		t.Fatalf("health band = %s, want yellow; audit=%#v", audit.HealthBand, audit)
	}
	if len(audit.HardBlockers) != 0 {
		t.Fatalf("hard blockers = %#v, want none", audit.HardBlockers)
	}
	if len(audit.ReviewOnlyFindings) == 0 {
		t.Fatalf("review-only findings missing; audit=%#v", audit)
	}
	if audit.Proceed.SimAutoContinueAllowed || audit.Proceed.RealOperatorConfirmAllowed {
		t.Fatalf("yellow proceed should not continue/confirm: %#v", audit.Proceed)
	}
}

func TestBuildHoverHealthAuditClassifiesSafetyBlockerAsRed(t *testing.T) {
	dir := writeHoverHealthArtifact(t, []map[string]any{{"code": "external_nav_loss_after_airborne", "message": "loss exceeded grace"}})
	audit, err := BuildHoverHealthAudit(dir)
	if err != nil {
		t.Fatal(err)
	}
	if audit.HealthBand != HoverHealthRed {
		t.Fatalf("health band = %s, want red; audit=%#v", audit.HealthBand, audit)
	}
	if len(audit.HardBlockers) == 0 {
		t.Fatalf("hard blockers missing; audit=%#v", audit)
	}
	if audit.Proceed.SimAutoContinueAllowed || audit.Proceed.RealOperatorConfirmAllowed {
		t.Fatalf("red proceed should not continue/confirm: %#v", audit.Proceed)
	}
}

func TestBuildHoverHealthCohortAggregatesRunBandsAndMetrics(t *testing.T) {
	greenish := writeHoverHealthArtifact(t, nil)
	yellow := writeHoverHealthArtifact(t, []map[string]any{{"code": "hover_gazebo_model_horizontal_drift"}})
	cohort, err := BuildHoverHealthCohort([]string{greenish, yellow})
	if err != nil {
		t.Fatal(err)
	}
	if cohort.SampleSize != 2 || cohort.SampleSizeRule != "case_study_only" {
		t.Fatalf("sample metadata = %#v", cohort)
	}
	if cohort.BandCounts[string(HoverHealthYellow)] == 0 {
		t.Fatalf("yellow band count missing: %#v", cohort.BandCounts)
	}
	if len(cohort.Metrics) == 0 {
		t.Fatalf("cohort metrics missing: %#v", cohort)
	}
}

func TestAttachHoverHealthToLiveRunSummaryAddsGatePayloadFields(t *testing.T) {
	dir := writeHoverHealthArtifact(t, []map[string]any{{"code": "hover_gazebo_model_horizontal_drift"}})
	health, path, err := BuildAndWriteHoverHealthSummaryArtifact(dir)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(path); err != nil {
		t.Fatalf("health summary artifact missing: %v", err)
	}
	summary := LiveRunSummary{ArtifactDir: dir, Metrics: map[string]any{}, Evidence: map[string]any{}}
	AttachHoverHealthToLiveRunSummary(&summary, health)
	if summary.HoverHealthBand != string(HoverHealthYellow) {
		t.Fatalf("summary hover health band = %q", summary.HoverHealthBand)
	}
	if summary.HoverHealthProceed == nil || summary.HoverHealthProceed.SimAutoContinueAllowed {
		t.Fatalf("hover health proceed = %#v", summary.HoverHealthProceed)
	}
	if summary.PostrunHoverHealthAudit == nil || summary.PostrunHoverHealthAudit.ControlsRuntimeProceed {
		t.Fatalf("postrun hover health audit missing/says it controls runtime: %#v", summary.PostrunHoverHealthAudit)
	}
	if summary.GateEvaluation.Metrics.HoverHealth["band"] != HoverHealthYellow {
		t.Fatalf("gate hover health metric missing: %#v", summary.GateEvaluation.Metrics.HoverHealth)
	}
	if summary.Metrics["hover_health"] == nil || summary.Evidence["hoverHealth"] == nil {
		t.Fatalf("summary hover health fields missing: metrics=%#v evidence=%#v", summary.Metrics, summary.Evidence)
	}
}

func TestAttachRuntimeHoverHealthFromMissionSummaryAddsRuntimeSchema(t *testing.T) {
	dir := t.TempDir()
	mission := map[string]any{
		"runtime_hover_health_final": map[string]any{
			"schema":                    "navlab.runtime_hover_health.v1",
			"phase":                     "sim_auto_continue",
			"band":                      "green",
			"sim_auto_continue_allowed": true,
		},
	}
	data, err := json.Marshal(mission)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "mission_summary.json"), data, 0o644); err != nil {
		t.Fatal(err)
	}
	summary := LiveRunSummary{ArtifactDir: dir, Metrics: map[string]any{}, Evidence: map[string]any{}}

	AttachRuntimeHoverHealthFromMissionSummary(&summary)

	if summary.RuntimeHoverHealthFinal["phase"] != "sim_auto_continue" {
		t.Fatalf("runtime hover health final missing: %#v", summary.RuntimeHoverHealthFinal)
	}
	if summary.Metrics["runtime_hover_health"] == nil || summary.Evidence["runtimeHoverHealth"] == nil {
		t.Fatalf("runtime health summary fields missing: metrics=%#v evidence=%#v", summary.Metrics, summary.Evidence)
	}
}

func writeHoverHealthArtifact(t *testing.T, blockers []map[string]any) string {
	t.Helper()
	dir := t.TempDir()
	bagDir := filepath.Join(dir, "rosbag", "hover_rosbag")
	if err := os.MkdirAll(bagDir, 0o755); err != nil {
		t.Fatal(err)
	}
	writeHoverInitializationAuditMCAP(t, filepath.Join(bagDir, "hover_rosbag_0.mcap"))
	summary := map[string]any{"ok": len(blockers) == 0, "status": "TASK_STATUS_OK", "blockers": blockers}
	if len(blockers) > 0 {
		summary["ok"] = false
		summary["status"] = "TASK_STATUS_BLOCKED"
	}
	data, err := json.Marshal(summary)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "summary.json"), data, 0o644); err != nil {
		t.Fatal(err)
	}
	return dir
}
