package foxglove

import (
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"

	"github.com/foxglove/mcap/go/mcap"
)

const (
	defaultDisplayTFSourceTopic = "/gazebo/model/odometry"
	defaultDisplayTFOutputTopic = "/tf"
	defaultDisplayTFParentFrame = "map"
	defaultDisplayTFChildFrame  = "base_link"
	displayTFCoordinateRaw      = "raw"
	displayTFGazeboXYZToNED     = "gazebo_xyz_to_ned"
)

type derivedTFProfile struct {
	Topic          string `json:"topic"`
	Source         string `json:"source"`
	Parent         string `json:"parent"`
	Child          string `json:"child"`
	Mode           string `json:"mode"`
	CoordinateMode string `json:"coordinate_mode,omitempty"`
	Role           string `json:"role"`
}

type derivedTFState struct {
	Spec           derivedTFProfile
	Messages       []derivedTFMessage
	SourceCount    int
	GeneratedCount int
}

type derivedTFMessage struct {
	LogTime     uint64
	PublishTime uint64
	Sequence    uint32
	StampSec    int32
	StampNsec   uint32
	Pose        odometryPose
}

type odometryPose struct {
	X  float64
	Y  float64
	Z  float64
	QX float64
	QY float64
	QZ float64
	QW float64
}

func normalizeDerivedTFProfile(spec derivedTFProfile) derivedTFProfile {
	if spec.Source == "" {
		spec.Source = defaultDisplayTFSourceTopic
	}
	if spec.Topic == "" {
		spec.Topic = defaultDisplayTFOutputTopic
	}
	if spec.Parent == "" {
		spec.Parent = defaultDisplayTFParentFrame
	}
	if spec.Child == "" {
		spec.Child = defaultDisplayTFChildFrame
	}
	if spec.Mode == "" {
		spec.Mode = "replace"
	}
	if spec.CoordinateMode == "" {
		spec.CoordinateMode = displayTFCoordinateRaw
	}
	if spec.Role == "" {
		spec.Role = "visualization_only"
	}
	return spec
}

func newDerivedTFStates(profile liteTopicProfile) []*derivedTFState {
	states := make([]*derivedTFState, 0, len(profile.DerivedTFs))
	for _, spec := range profile.DerivedTFs {
		states = append(states, &derivedTFState{Spec: normalizeDerivedTFProfile(spec)})
	}
	return states
}

func derivedTFReplaceEdges(states []*derivedTFState) map[string][][2]string {
	edges := map[string][][2]string{}
	for _, state := range states {
		if state.Spec.Mode == "replace" {
			edges[state.Spec.Topic] = append(edges[state.Spec.Topic], [2]string{state.Spec.Parent, state.Spec.Child})
		}
	}
	return edges
}

func (state *derivedTFState) maybeCollect(channel *mcap.Channel, message *mcap.Message) error {
	if state == nil || channel == nil || message == nil || channel.Topic != state.Spec.Source {
		return nil
	}
	state.SourceCount++
	pose, stampSec, stampNsec, err := parseOdometryPoseCDR(message.Data)
	if err != nil {
		return fmt.Errorf("parse display odometry source %s: %w", state.Spec.Source, err)
	}
	state.Messages = append(state.Messages, derivedTFMessage{
		LogTime:     message.LogTime,
		PublishTime: message.PublishTime,
		Sequence:    message.Sequence,
		StampSec:    stampSec,
		StampNsec:   stampNsec,
		Pose:        pose,
	})
	return nil
}

func writeDerivedTFOutputs(writer *mcap.Writer, states []*derivedTFState, nextSchemaID uint16, nextChannelID uint16) ([]derivedTFProfile, error) {
	var summaries []derivedTFProfile
	for _, state := range states {
		if state.SourceCount == 0 {
			return nil, fmt.Errorf("derived display TF source topic missing: %s", state.Spec.Source)
		}
		if len(state.Messages) == 0 {
			return nil, fmt.Errorf("derived display TF unavailable for %s", state.Spec.Topic)
		}
		schemaID := nextSchemaID
		channelID := nextChannelID
		if err := writer.WriteSchema(tfMessageSchema(schemaID)); err != nil {
			return nil, err
		}
		if err := writer.WriteChannel(&mcap.Channel{ID: channelID, SchemaID: schemaID, Topic: state.Spec.Topic, MessageEncoding: "cdr"}); err != nil {
			return nil, err
		}
		nextSchemaID++
		nextChannelID++
		for _, message := range state.Messages {
			pose, err := transformDerivedTFPose(message.Pose, state.Spec.CoordinateMode)
			if err != nil {
				return nil, err
			}
			if err := writer.WriteMessage(&mcap.Message{
				ChannelID:   channelID,
				Sequence:    message.Sequence,
				LogTime:     message.LogTime,
				PublishTime: message.PublishTime,
				Data:        encodeTransformTFMessagePose(state.Spec.Parent, state.Spec.Child, pose, message.StampSec, message.StampNsec),
			}); err != nil {
				return nil, err
			}
			state.GeneratedCount++
		}
		summaries = append(summaries, state.Spec)
	}
	return summaries, nil
}

func transformDerivedTFPose(pose odometryPose, mode string) (odometryPose, error) {
	switch mode {
	case "", displayTFCoordinateRaw:
		return pose, nil
	case displayTFGazeboXYZToNED:
		yaw := yawFromQuaternion(pose.QX, pose.QY, pose.QZ, pose.QW)
		mappedYaw := yaw - math.Pi/2
		qz, qw := yawQuaternion(mappedYaw)
		return odometryPose{
			X:  pose.Y,
			Y:  -pose.X,
			Z:  pose.Z,
			QX: 0,
			QY: 0,
			QZ: qz,
			QW: qw,
		}, nil
	default:
		return odometryPose{}, fmt.Errorf("unknown derived display TF coordinate mode %q", mode)
	}
}

func yawFromQuaternion(qx float64, qy float64, qz float64, qw float64) float64 {
	return math.Atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
}

func yawQuaternion(yaw float64) (float64, float64) {
	half := yaw / 2
	return math.Sin(half), math.Cos(half)
}

func parseOdometryPoseCDR(data []byte) (odometryPose, int32, uint32, error) {
	cursor := cdrCursor{data: data}
	if err := cursor.skip(4); err != nil {
		return odometryPose{}, 0, 0, err
	}
	stampSec, err := cursor.int32()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	stampNsec, err := cursor.uint32()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	if err := cursor.string(); err != nil {
		return odometryPose{}, 0, 0, err
	}
	if err := cursor.string(); err != nil {
		return odometryPose{}, 0, 0, err
	}
	x, err := cursor.float64()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	y, err := cursor.float64()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	z, err := cursor.float64()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	qx, err := cursor.float64()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	qy, err := cursor.float64()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	qz, err := cursor.float64()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	qw, err := cursor.float64()
	if err != nil {
		return odometryPose{}, 0, 0, err
	}
	if !finitePose(x, y, z, qx, qy, qz, qw) {
		return odometryPose{}, 0, 0, errors.New("odometry pose contains non-finite values")
	}
	return odometryPose{X: x, Y: y, Z: z, QX: qx, QY: qy, QZ: qz, QW: qw}, stampSec, stampNsec, nil
}

func finitePose(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func (cursor *cdrCursor) float64() (float64, error) {
	cursor.align(8)
	if cursor.off+8 > len(cursor.data) {
		return 0, io.ErrUnexpectedEOF
	}
	value := math.Float64frombits(binary.LittleEndian.Uint64(cursor.data[cursor.off : cursor.off+8]))
	cursor.off += 8
	return value, nil
}

func encodeTransformTFMessagePose(parent string, child string, pose odometryPose, stampSec int32, stampNsec uint32) []byte {
	builder := newCDRBuilder()
	builder.uint32(1) // transforms length
	builder.int32(stampSec)
	builder.uint32(stampNsec)
	builder.string(parent)
	builder.string(child)
	builder.float64(pose.X)
	builder.float64(pose.Y)
	builder.float64(pose.Z)
	builder.float64(pose.QX)
	builder.float64(pose.QY)
	builder.float64(pose.QZ)
	builder.float64(pose.QW)
	return builder.bytes()
}

type decodedTransform struct {
	StampSec  int32
	StampNsec uint32
	Parent    string
	Child     string
	X         float64
	Y         float64
	Z         float64
	QX        float64
	QY        float64
	QZ        float64
	QW        float64
}

func filterTFMessageCDR(data []byte, replaceEdges [][2]string) ([]byte, bool, error) {
	transforms, err := decodeTFMessageCDR(data)
	if err != nil {
		return nil, false, err
	}
	kept := make([]decodedTransform, 0, len(transforms))
	for _, transform := range transforms {
		if transformMatchesAnyEdge(transform, replaceEdges) {
			continue
		}
		kept = append(kept, transform)
	}
	if len(kept) == 0 {
		return nil, false, nil
	}
	return encodeTFMessageCDR(kept), true, nil
}

func decodeTFMessageCDR(data []byte) ([]decodedTransform, error) {
	cursor := cdrCursor{data: data}
	if err := cursor.skip(4); err != nil {
		return nil, err
	}
	count, err := cursor.uint32()
	if err != nil {
		return nil, err
	}
	transforms := make([]decodedTransform, 0, count)
	for index := uint32(0); index < count; index++ {
		stampSec, err := cursor.int32()
		if err != nil {
			return nil, err
		}
		stampNsec, err := cursor.uint32()
		if err != nil {
			return nil, err
		}
		parent, err := cursor.stringValue()
		if err != nil {
			return nil, err
		}
		child, err := cursor.stringValue()
		if err != nil {
			return nil, err
		}
		x, err := cursor.float64()
		if err != nil {
			return nil, err
		}
		y, err := cursor.float64()
		if err != nil {
			return nil, err
		}
		z, err := cursor.float64()
		if err != nil {
			return nil, err
		}
		qx, err := cursor.float64()
		if err != nil {
			return nil, err
		}
		qy, err := cursor.float64()
		if err != nil {
			return nil, err
		}
		qz, err := cursor.float64()
		if err != nil {
			return nil, err
		}
		qw, err := cursor.float64()
		if err != nil {
			return nil, err
		}
		transforms = append(transforms, decodedTransform{
			StampSec:  stampSec,
			StampNsec: stampNsec,
			Parent:    parent,
			Child:     child,
			X:         x,
			Y:         y,
			Z:         z,
			QX:        qx,
			QY:        qy,
			QZ:        qz,
			QW:        qw,
		})
	}
	return transforms, nil
}

func encodeTFMessageCDR(transforms []decodedTransform) []byte {
	builder := newCDRBuilder()
	builder.uint32(uint32(len(transforms)))
	for _, transform := range transforms {
		builder.int32(transform.StampSec)
		builder.uint32(transform.StampNsec)
		builder.string(transform.Parent)
		builder.string(transform.Child)
		builder.float64(transform.X)
		builder.float64(transform.Y)
		builder.float64(transform.Z)
		builder.float64(transform.QX)
		builder.float64(transform.QY)
		builder.float64(transform.QZ)
		builder.float64(transform.QW)
	}
	return builder.bytes()
}

func transformMatchesAnyEdge(transform decodedTransform, edges [][2]string) bool {
	for _, edge := range edges {
		if transform.Parent == edge[0] && transform.Child == edge[1] {
			return true
		}
	}
	return false
}
