package helpers

import (
	"encoding/json"
	"encoding/xml"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"strings"
)

const OfficialMazeOverlayTopic = "/navlab/official_maze/map"

type OfficialMazeOverlaySpec struct {
	Topic       string
	AliasTopics []string
	FrameID     string
	ResolutionM float64
	MarginM     float64
}

type OfficialMazeWall struct {
	Name      string  `json:"name"`
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	Yaw       float64 `json:"yaw"`
	Length    float64 `json:"length"`
	Thickness float64 `json:"thickness"`
}

func DefaultOfficialMazeOverlaySpec() OfficialMazeOverlaySpec {
	return OfficialMazeOverlaySpec{
		Topic:       OfficialMazeOverlayTopic,
		FrameID:     "map",
		ResolutionM: 0.10,
		MarginM:     0.20,
	}
}

func WriteOfficialMazeOverlayRuntimeScript(path string, source string, spec OfficialMazeOverlaySpec) error {
	if strings.TrimSpace(spec.Topic) == "" {
		spec.Topic = OfficialMazeOverlayTopic
	}
	if strings.TrimSpace(spec.FrameID) == "" {
		spec.FrameID = "map"
	}
	if spec.ResolutionM <= 0 {
		spec.ResolutionM = 0.10
	}
	if spec.MarginM < 0 {
		spec.MarginM = 0
	}
	topics := append([]string{spec.Topic}, spec.AliasTopics...)
	walls, err := ParseOfficialMazeWalls(source)
	if err != nil {
		return err
	}
	if len(walls) == 0 {
		return fmt.Errorf("official maze overlay has no walls")
	}
	payload, err := json.Marshal(map[string]any{
		"topic":        spec.Topic,
		"topics":       topics,
		"frame_id":     spec.FrameID,
		"resolution_m": spec.ResolutionM,
		"margin_m":     spec.MarginM,
		"walls":        walls,
	})
	if err != nil {
		return err
	}
	script := fmt.Sprintf(`#!/usr/bin/env python3
import json
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


SPEC = json.loads(r'''%s''')


def wall_corners(wall):
    half_l = wall["length"] / 2.0
    half_t = wall["thickness"] / 2.0
    c = math.cos(wall["yaw"])
    s = math.sin(wall["yaw"])
    points = []
    for lx, ly in [(-half_l, -half_t), (half_l, -half_t), (half_l, half_t), (-half_l, half_t)]:
        points.append((wall["x"] + c * lx - s * ly, wall["y"] + s * lx + c * ly))
    return points


def point_inside_wall(x, y, wall):
    dx = x - wall["x"]
    dy = y - wall["y"]
    c = math.cos(wall["yaw"])
    s = math.sin(wall["yaw"])
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return abs(lx) <= wall["length"] / 2.0 and abs(ly) <= wall["thickness"] / 2.0


def build_map():
    walls = SPEC["walls"]
    margin = float(SPEC["margin_m"])
    resolution = float(SPEC["resolution_m"])
    points = [point for wall in walls for point in wall_corners(wall)]
    xmin = min(point[0] for point in points) - margin
    xmax = max(point[0] for point in points) + margin
    ymin = min(point[1] for point in points) - margin
    ymax = max(point[1] for point in points) + margin
    width = max(1, int(math.ceil((xmax - xmin) / resolution)))
    height = max(1, int(math.ceil((ymax - ymin) / resolution)))
    data = [-1] * (width * height)
    for row in range(height):
        y = ymin + (row + 0.5) * resolution
        for col in range(width):
            x = xmin + (col + 0.5) * resolution
            if any(point_inside_wall(x, y, wall) for wall in walls):
                data[row * width + col] = 100
            else:
                data[row * width + col] = 0
    msg = OccupancyGrid()
    msg.header.frame_id = SPEC["frame_id"]
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = xmin
    msg.info.origin.position.y = ymin
    msg.info.origin.orientation.w = 1.0
    msg.data = data
    return msg


def main():
    rclpy.init()
    node = rclpy.create_node("navlab_official_maze_overlay")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publishers = [node.create_publisher(OccupancyGrid, topic, qos) for topic in SPEC["topics"]]
    msg = build_map()

    def publish():
        msg.header.stamp = node.get_clock().now().to_msg()
        for publisher in publishers:
            publisher.publish(msg)

    timer = node.create_timer(1.0, publish)
    publish()
    node.get_logger().info(
        "official maze overlay publishing topics=%%s frame=%%s width=%%d height=%%d"
        %% (SPEC["topics"], SPEC["frame_id"], msg.info.width, msg.info.height)
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
`, string(payload))
	return os.WriteFile(path, []byte(script), 0o755)
}

func ParseOfficialMazeWalls(source string) ([]OfficialMazeWall, error) {
	decoder := xml.NewDecoder(strings.NewReader(source))
	for {
		token, err := decoder.Token()
		if errors.Is(err, io.EOF) {
			break
		}
		if err != nil {
			return nil, err
		}
		start, ok := token.(xml.StartElement)
		if !ok || start.Name.Local != "model" || officialMazeAttr(start.Attr, "name") != "maze" {
			continue
		}
		var model officialMazeSDFModel
		if err := decoder.DecodeElement(&model, &start); err != nil {
			return nil, err
		}
		return officialMazeWallsFromModel(model), nil
	}
	return nil, fmt.Errorf("maze model not found in official SDF")
}

type officialMazeSDFModel struct {
	Links []officialMazeSDFLink `xml:"link"`
}

type officialMazeSDFLink struct {
	Name      string                    `xml:"name,attr"`
	Pose      officialMazeSDFPose       `xml:"pose"`
	Collision *officialMazeSDFCollision `xml:"collision"`
}

type officialMazeSDFPose struct {
	Degrees string `xml:"degrees,attr"`
	Text    string `xml:",chardata"`
}

type officialMazeSDFCollision struct {
	Pose     officialMazeSDFPose `xml:"pose"`
	Geometry struct {
		Box struct {
			Size string `xml:"size"`
		} `xml:"box"`
	} `xml:"geometry"`
}

func officialMazeWallsFromModel(model officialMazeSDFModel) []OfficialMazeWall {
	var walls []OfficialMazeWall
	for _, link := range model.Links {
		if link.Collision == nil {
			continue
		}
		linkPose := officialMazePose(link.Pose.Text)
		linkYaw := linkPose[5]
		if link.Pose.Degrees == "true" {
			linkYaw = linkYaw * math.Pi / 180
		}
		collisionPose := officialMazePose(link.Collision.Pose.Text)
		size := officialMazeFloats(link.Collision.Geometry.Box.Size)
		if len(size) < 3 {
			continue
		}
		cx, cy := collisionPose[0], collisionPose[1]
		walls = append(walls, OfficialMazeWall{
			Name:      link.Name,
			X:         linkPose[0] + math.Cos(linkYaw)*cx - math.Sin(linkYaw)*cy,
			Y:         linkPose[1] + math.Sin(linkYaw)*cx + math.Cos(linkYaw)*cy,
			Yaw:       linkYaw,
			Length:    size[0],
			Thickness: size[1],
		})
	}
	return walls
}

func officialMazePose(value string) [6]float64 {
	values := officialMazeFloats(value)
	var result [6]float64
	for i := range result {
		if i < len(values) {
			result[i] = values[i]
		}
	}
	return result
}

func officialMazeFloats(value string) []float64 {
	fields := strings.Fields(value)
	values := make([]float64, 0, len(fields))
	for _, field := range fields {
		var parsed float64
		if _, err := fmt.Sscan(field, &parsed); err == nil {
			values = append(values, parsed)
		}
	}
	return values
}

func officialMazeAttr(attrs []xml.Attr, name string) string {
	for _, attr := range attrs {
		if attr.Name.Local == name {
			return attr.Value
		}
	}
	return ""
}
