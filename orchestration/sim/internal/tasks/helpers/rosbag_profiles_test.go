package helpers

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestProfileTopics(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "topics.txt")
	if err := os.WriteFile(path, []byte("# comment\nrequired /tf interval=0.1\noptional /scan\nignore /x\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	topics, err := ProfileTopics(path)
	if err != nil {
		t.Fatalf("ProfileTopics() error = %v", err)
	}
	if len(topics.Required) != 1 || topics.Required[0] != "/tf" {
		t.Fatalf("required = %#v, want /tf", topics.Required)
	}
	if len(topics.Optional) != 1 || topics.Optional[0] != "/scan" {
		t.Fatalf("optional = %#v, want /scan", topics.Optional)
	}
}

func TestMetadataCountsFromString(t *testing.T) {
	counts := MetadataCountsFromString(`
topics_with_message_count:
  - topic_metadata:
      name: /tf
    message_count: 4
  - topic_metadata:
      name: /scan
    message_count: 0
`)
	if counts["/tf"] != 4 {
		t.Fatalf("/tf count = %d, want 4", counts["/tf"])
	}
	if counts["/scan"] != 0 {
		t.Fatalf("/scan count = %d, want 0", counts["/scan"])
	}
}

func TestNavigationRosbagProfileKeepsDiagnosticTruthReviewOnly(t *testing.T) {
	data, err := os.ReadFile("../../../configs/rosbag/navigation.yaml")
	if err != nil {
		t.Fatal(err)
	}
	required := topicListFromYAMLSection(string(data), "required_topics")
	review := topicListFromYAMLSection(string(data), "review_topics")
	if contains(required, DiagnosticTruthOdometryTopic) {
		t.Fatalf("navigation profile requires diagnostic truth %q: %#v", DiagnosticTruthOdometryTopic, required)
	}
	if !contains(review, DiagnosticTruthOdometryTopic) {
		t.Fatalf("navigation profile should keep %q as review-only: %#v", DiagnosticTruthOdometryTopic, review)
	}
	if contains(required, DiagnosticGazeboModelOdometryTopic) {
		t.Fatalf("navigation profile requires Gazebo model odometry %q: %#v", DiagnosticGazeboModelOdometryTopic, required)
	}
	if !contains(review, DiagnosticGazeboModelOdometryTopic) {
		t.Fatalf("navigation profile should keep %q as review-only: %#v", DiagnosticGazeboModelOdometryTopic, review)
	}
}

func topicListFromYAMLSection(content string, section string) []string {
	var topics []string
	inSection := false
	for _, rawLine := range strings.Split(content, "\n") {
		line := strings.TrimSpace(rawLine)
		switch {
		case line == section+":":
			inSection = true
		case inSection && strings.HasSuffix(line, ":"):
			return topics
		case inSection && strings.HasPrefix(line, "- "):
			topics = append(topics, strings.TrimSpace(strings.TrimPrefix(line, "- ")))
		}
	}
	return topics
}
