package foxglove

import (
	"bytes"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/foxglove/mcap/go/mcap"
	"github.com/klauspost/compress/zstd"
)

func TestBuildReplayRejectsRawMissingOfficialMazeOverlay(t *testing.T) {
	runDir := makeRun(t, "hover", "20260615T010000Z")
	mustWriteJSON(t, filepath.Join(runDir, "summary.json"), validTaskSummary("hover"))
	writeTestMCAP(t, filepath.Join(runDir, rawMCAPRelative("hover")), []string{"/tf", "/tf_static", "/map", "/scan", "/slam/odom"})
	mustWriteLiteProfile(t, runDir, "hover", `
overlay /navlab/official_maze/map
required /tf interval=all
required /tf_static interval=all
required /map interval=all
required /scan interval=all
required /slam/odom interval=all
`)

	_, err := BuildReplay(ReplayOptions{
		RepoRoot:     repoRootFromRunDir(runDir),
		ArtifactRoot: "artifacts/sim",
		Task:         "hover",
		Run:          filepath.Base(runDir),
	})
	if err == nil || !strings.Contains(err.Error(), "/navlab/official_maze/map") {
		t.Fatalf("err = %v", err)
	}
}

func TestBuildReplayCopiesProfileTopicsWithMCAPLibrary(t *testing.T) {
	runDir := makeRun(t, "hover", "20260615T020000Z")
	mustWriteJSON(t, filepath.Join(runDir, "summary.json"), validTaskSummary("hover"))
	writeTestMCAP(t, filepath.Join(runDir, rawMCAPRelative("hover")), []string{
		"/navlab/official_maze/map",
		"/tf",
		"/tf_static",
		"/map",
		"/scan",
		"/slam/odom",
		"/clock",
	})
	mustWriteLiteProfile(t, runDir, "hover", `
overlay /navlab/official_maze/map
required /tf interval=all
required /tf_static interval=all
required /map interval=all
required /scan interval=all
required /slam/odom interval=all
drop /clock
`)

	result, err := BuildReplay(ReplayOptions{
		RepoRoot:     repoRootFromRunDir(runDir),
		ArtifactRoot: "artifacts/sim",
		Task:         "hover",
		Run:          filepath.Base(runDir),
	})
	if err != nil {
		t.Fatal(err)
	}
	if result.ReplayMCAPInfo.MessageCounts["/navlab/official_maze/map"] != 1 {
		t.Fatalf("message counts = %#v", result.ReplayMCAPInfo.MessageCounts)
	}
	if result.TaskSummary.Path == "" || !result.TaskSummary.OK {
		t.Fatalf("task summary gate = %#v", result.TaskSummary)
	}
	if result.ReplayMCAPInfo.MessageCounts["/clock"] != 0 {
		t.Fatalf("drop topic leaked into lite: %#v", result.ReplayMCAPInfo.MessageCounts)
	}
	targets, err := BuildTargets(runDir, "hover", true, "navlab/sim")
	if err != nil {
		t.Fatal(err)
	}
	if len(targets) == 0 || !strings.Contains(targets[0].Filename, "_lite_") {
		t.Fatalf("targets = %#v", targets)
	}
}

func TestBuildReplayReadsCompressedRawMCAPWithoutPlainCopy(t *testing.T) {
	runDir := makeRun(t, "hover", "20260615T030000Z")
	mustWriteJSON(t, filepath.Join(runDir, "summary.json"), validTaskSummary("hover"))
	rawPath := filepath.Join(runDir, rawMCAPRelative("hover"))
	writeTestCompressedMCAP(t, rawPath+".zstd", []string{
		"/navlab/official_maze/map",
		"/tf",
		"/tf_static",
		"/map",
		"/scan",
		"/slam/odom",
	})
	mustWriteLiteProfile(t, runDir, "hover", `
overlay /navlab/official_maze/map
required /tf interval=all
required /tf_static interval=all
required /map interval=all
required /scan interval=all
required /slam/odom interval=all
`)

	result, err := BuildReplay(ReplayOptions{
		RepoRoot:     repoRootFromRunDir(runDir),
		ArtifactRoot: "artifacts/sim",
		Task:         "hover",
		Run:          filepath.Base(runDir),
	})
	if err != nil {
		t.Fatal(err)
	}
	if result.RawMCAP != rawPath+".zstd" {
		t.Fatalf("raw path = %q, want compressed path", result.RawMCAP)
	}
	if _, err := os.Stat(rawPath); !os.IsNotExist(err) {
		t.Fatalf("plain raw MCAP should not be created, stat err = %v", err)
	}
	if result.ReplayMCAPInfo.MessageCounts["/navlab/official_maze/map"] != 1 {
		t.Fatalf("message counts = %#v", result.ReplayMCAPInfo.MessageCounts)
	}
}

func TestBuildReplayRejectsBlockedTaskSummary(t *testing.T) {
	runDir := makeRun(t, "hover", "20260615T040000Z")
	mustWriteJSON(t, filepath.Join(runDir, "summary.json"), map[string]any{
		"task_id":      "hover",
		"ok":           false,
		"blocked":      true,
		"blockerCodes": []string{"hover_takeoff_target_altitude_not_reached"},
	})
	writeTestMCAP(t, filepath.Join(runDir, rawMCAPRelative("hover")), []string{
		"/navlab/official_maze/map",
		"/tf",
		"/tf_static",
		"/map",
		"/scan",
		"/slam/odom",
	})
	mustWriteLiteProfile(t, runDir, "hover", `
overlay /navlab/official_maze/map
required /tf interval=all
required /tf_static interval=all
required /map interval=all
required /scan interval=all
required /slam/odom interval=all
`)

	_, err := BuildReplay(ReplayOptions{
		RepoRoot:     repoRootFromRunDir(runDir),
		ArtifactRoot: "artifacts/sim",
		Task:         "hover",
		Run:          filepath.Base(runDir),
	})
	if err == nil || !strings.Contains(err.Error(), "task summary is not ok") {
		t.Fatalf("err = %v", err)
	}
}

func writeTestMCAP(t *testing.T, path string, topics []string) {
	t.Helper()
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatal(err)
	}
	file, err := os.Create(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	writer, err := mcap.NewWriter(file, &mcap.WriterOptions{Chunked: true})
	if err != nil {
		t.Fatal(err)
	}
	if err := writer.WriteHeader(&mcap.Header{Profile: "ros2", Library: "test"}); err != nil {
		t.Fatal(err)
	}
	schema := &mcap.Schema{ID: 1, Name: "std_msgs/msg/String", Encoding: "ros2msg", Data: []byte("string data\n")}
	if err := writer.WriteSchema(schema); err != nil {
		t.Fatal(err)
	}
	for index, topic := range topics {
		channelID := uint16(index + 1)
		if err := writer.WriteChannel(&mcap.Channel{
			ID:              channelID,
			SchemaID:        schema.ID,
			Topic:           topic,
			MessageEncoding: "cdr",
		}); err != nil {
			t.Fatal(err)
		}
		if err := writer.WriteMessage(&mcap.Message{
			ChannelID:   channelID,
			LogTime:     uint64(index+1) * 1_000_000,
			PublishTime: uint64(index+1) * 1_000_000,
			Data:        []byte{0, 1, byte(index)},
		}); err != nil {
			t.Fatal(err)
		}
	}
	if err := writer.Close(); err != nil {
		t.Fatal(err)
	}
}

func writeTestCompressedMCAP(t *testing.T, path string, topics []string) {
	t.Helper()
	var plain bytes.Buffer
	writeTestMCAPTo(t, &plain, topics)
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatal(err)
	}
	file, err := os.Create(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	encoder, err := zstd.NewWriter(file)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := encoder.Write(plain.Bytes()); err != nil {
		t.Fatal(err)
	}
	if err := encoder.Close(); err != nil {
		t.Fatal(err)
	}
}

func writeTestMCAPTo(t *testing.T, output *bytes.Buffer, topics []string) {
	t.Helper()
	writer, err := mcap.NewWriter(output, &mcap.WriterOptions{Chunked: true})
	if err != nil {
		t.Fatal(err)
	}
	writeTestMCAPContents(t, writer, topics)
}

func writeTestMCAPContents(t *testing.T, writer *mcap.Writer, topics []string) {
	t.Helper()
	if err := writer.WriteHeader(&mcap.Header{Profile: "ros2", Library: "test"}); err != nil {
		t.Fatal(err)
	}
	schema := &mcap.Schema{ID: 1, Name: "std_msgs/msg/String", Encoding: "ros2msg", Data: []byte("string data\n")}
	if err := writer.WriteSchema(schema); err != nil {
		t.Fatal(err)
	}
	for index, topic := range topics {
		channelID := uint16(index + 1)
		if err := writer.WriteChannel(&mcap.Channel{
			ID:              channelID,
			SchemaID:        schema.ID,
			Topic:           topic,
			MessageEncoding: "cdr",
		}); err != nil {
			t.Fatal(err)
		}
		if err := writer.WriteMessage(&mcap.Message{
			ChannelID:   channelID,
			LogTime:     uint64(index+1) * 1_000_000,
			PublishTime: uint64(index+1) * 1_000_000,
			Data:        []byte{0, 1, byte(index)},
		}); err != nil {
			t.Fatal(err)
		}
	}
	if err := writer.Close(); err != nil {
		t.Fatal(err)
	}
}
