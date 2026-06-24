package hover

import (
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/foxglove/mcap/go/mcap"
)

func TestSummarizeHoverContractAuditRecordsFramesStatusAndTFPairs(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "hover_rosbag_0.mcap")
	writeHoverContractAuditMCAP(t, path)

	audit, err := summarizeHoverContractAudit(path, dir)
	if err != nil {
		t.Fatal(err)
	}

	topics := mapFromAny(audit["topics"])
	slam := mapFromAny(topics["slam_odom"])
	slamObserved := mapFromAny(slam["observed"])
	if got := contractCountForValue(slamObserved, "frame_counts", "map"); got != 2 {
		t.Fatalf("slam map frame count = %d, want 2; observed=%#v", got, slamObserved)
	}
	if slamObserved["frame_contract_status"] != "observed_expected" {
		t.Fatalf("slam frame contract status = %#v", slamObserved["frame_contract_status"])
	}

	mav := mapFromAny(topics["mavlink_odometry_status"])
	mavObserved := mapFromAny(mav["observed"])
	if got := contractCountForValue(mavObserved, "frame_counts", "MAV_FRAME_LOCAL_FRD"); got != 1 {
		t.Fatalf("mav frame count = %d, want 1; observed=%#v", got, mavObserved)
	}
	latest := mapFromAny(mavObserved["latest_status"])
	if latest["quality"] != float64(100) {
		t.Fatalf("latest mav status = %#v", latest)
	}

	gazeboTF := mapFromAny(topics["gazebo_tf"])
	tfObserved := mapFromAny(gazeboTF["observed"])
	if got := contractCountForValue(tfObserved, "transform_pairs", "world->navlab_iq_quad::base_link"); got != 1 {
		t.Fatalf("tf pair count = %d, want 1; observed=%#v", got, tfObserved)
	}
}

func writeHoverContractAuditMCAP(t *testing.T, path string) {
	t.Helper()
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
	odomSchema := &mcap.Schema{ID: 1, Name: "nav_msgs/msg/Odometry", Encoding: "ros2msg", Data: []byte("std_msgs/Header header\nstring child_frame_id\n")}
	stringSchema := &mcap.Schema{ID: 2, Name: "std_msgs/msg/String", Encoding: "ros2msg", Data: []byte("string data\n")}
	tfSchema := &mcap.Schema{ID: 3, Name: "tf2_msgs/msg/TFMessage", Encoding: "ros2msg", Data: []byte("geometry_msgs/TransformStamped[] transforms\n")}
	for _, schema := range []*mcap.Schema{odomSchema, stringSchema, tfSchema} {
		if err := writer.WriteSchema(schema); err != nil {
			t.Fatal(err)
		}
	}
	channels := []*mcap.Channel{
		{ID: 1, SchemaID: 1, Topic: "/slam/odom", MessageEncoding: "cdr"},
		{ID: 2, SchemaID: 2, Topic: "/mavlink_external_nav/status", MessageEncoding: "cdr"},
		{ID: 3, SchemaID: 3, Topic: "/gazebo/tf", MessageEncoding: "cdr"},
	}
	for _, channel := range channels {
		if err := writer.WriteChannel(channel); err != nil {
			t.Fatal(err)
		}
	}
	writeMsg := func(channel uint16, sec uint64, data []byte) {
		t.Helper()
		stamp := sec * 1_000_000_000
		if err := writer.WriteMessage(&mcap.Message{ChannelID: channel, LogTime: stamp, PublishTime: stamp, Data: data}); err != nil {
			t.Fatal(err)
		}
	}
	writeMsg(1, 1, gateTestOdometryPoseCDR(0.0, 0.0, 0.0, 0.0))
	writeMsg(1, 2, gateTestOdometryPoseCDR(0.1, 0.0, 0.0, 0.0))
	writeMsg(2, 2, gateTestStringCDR(`{"state":"sending","ready":true,"frame_id":"external_nav","child_frame_id":"base_link","mav_frame_id":"MAV_FRAME_LOCAL_FRD","mav_child_frame_id":"MAV_FRAME_BODY_FRD","quality":100}`))
	writeMsg(3, 2, gateTestTFMessageCDR("world", "navlab_iq_quad::base_link"))
	if err := writer.Close(); err != nil {
		t.Fatal(err)
	}
}

func gateTestTFMessageCDR(frameID string, childFrameID string) []byte {
	builder := gateTestCDRBuilder{data: []byte{0, 1, 0, 0}}
	builder.uint32(1)
	builder.int32(1)
	builder.uint32(2)
	builder.string(frameID)
	builder.string(childFrameID)
	builder.float64(0)
	builder.float64(0)
	builder.float64(0)
	builder.float64(0)
	builder.float64(0)
	builder.float64(math.Sin(0))
	builder.float64(math.Cos(0))
	return builder.data
}

func contractCountForValue(observed map[string]any, field string, value string) int {
	rows, _ := observed[field].([]map[string]any)
	for _, row := range rows {
		if row["value"] == value {
			return metricInt(row, "count")
		}
	}
	if rawRows, ok := observed[field].([]any); ok {
		for _, raw := range rawRows {
			row := mapFromAny(raw)
			if row["value"] == value {
				return metricInt(row, "count")
			}
		}
	}
	return 0
}
