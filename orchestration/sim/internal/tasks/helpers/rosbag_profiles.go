package helpers

import (
	"os"
	"regexp"
	"sort"
	"strconv"
	"strings"
)

type Topics struct {
	Required []string
	Optional []string
	All      []string
}

type Validation struct {
	OK                      bool           `json:"ok"`
	Recorded                bool           `json:"recorded"`
	Profile                 string         `json:"profile"`
	Metadata                string         `json:"metadata"`
	RequiredTopics          []string       `json:"required_topics"`
	OptionalTopics          []string       `json:"optional_topics"`
	PresentTopics           []string       `json:"present_topics"`
	MessageCounts           map[string]int `json:"message_counts"`
	MissingRequiredTopics   []string       `json:"missing_required_topics"`
	ZeroCountRequiredTopics []string       `json:"zero_count_required_topics"`
	PresentOptionalTopics   []string       `json:"present_optional_topics"`
	MissingOptionalTopics   []string       `json:"missing_optional_topics"`
}

func ProfileTopics(path string) (Topics, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return Topics{}, err
	}
	var required []string
	var optional []string
	for _, rawLine := range strings.Split(string(data), "\n") {
		line := strings.TrimSpace(rawLine)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		parts := strings.Fields(line)
		if len(parts) < 2 {
			continue
		}
		switch parts[0] {
		case "required":
			required = append(required, parts[1])
		case "optional":
			optional = append(optional, parts[1])
		}
	}
	all := append(append([]string{}, required...), optional...)
	return Topics{Required: required, Optional: optional, All: all}, nil
}

func LoadMetadataCounts(path string) (map[string]int, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	return MetadataCountsFromString(string(data)), nil
}

func MetadataCountsFromString(content string) map[string]int {
	counts := map[string]int{}
	topicPattern := regexp.MustCompile(`name: (/[^\n]+)`)
	countPattern := regexp.MustCompile(`message_count:\s*(\d+)`)
	matches := topicPattern.FindAllStringSubmatchIndex(content, -1)
	for index, match := range matches {
		topic := strings.TrimSpace(content[match[2]:match[3]])
		end := len(content)
		if index+1 < len(matches) {
			end = matches[index+1][0]
		}
		block := content[match[1]:end]
		countMatch := countPattern.FindStringSubmatch(block)
		if len(countMatch) != 2 {
			counts[topic] = 0
			continue
		}
		count, err := strconv.Atoi(countMatch[1])
		if err != nil {
			count = 0
		}
		counts[topic] = count
	}
	return counts
}

func ValidateProfile(profilePath string, metadataPath string, required []string, optional []string) (Validation, error) {
	counts, err := LoadMetadataCounts(metadataPath)
	if err != nil {
		return Validation{}, err
	}
	presentTopics := make([]string, 0, len(counts))
	for topic := range counts {
		presentTopics = append(presentTopics, topic)
	}
	sort.Strings(presentTopics)

	missingRequired := missingTopics(required, counts)
	zeroCountRequired := zeroCountTopics(required, counts)
	presentOptional := positiveCountTopics(optional, counts)
	missingOptional := missingTopics(optional, counts)

	return Validation{
		OK:                      len(missingRequired) == 0 && len(zeroCountRequired) == 0,
		Recorded:                true,
		Profile:                 profilePath,
		Metadata:                metadataPath,
		RequiredTopics:          append([]string{}, required...),
		OptionalTopics:          append([]string{}, optional...),
		PresentTopics:           presentTopics,
		MessageCounts:           counts,
		MissingRequiredTopics:   missingRequired,
		ZeroCountRequiredTopics: zeroCountRequired,
		PresentOptionalTopics:   presentOptional,
		MissingOptionalTopics:   missingOptional,
	}, nil
}

func missingTopics(topics []string, counts map[string]int) []string {
	var missing []string
	for _, topic := range topics {
		if _, exists := counts[topic]; !exists {
			missing = append(missing, topic)
		}
	}
	return missing
}

func zeroCountTopics(topics []string, counts map[string]int) []string {
	var zero []string
	for _, topic := range topics {
		if counts[topic] <= 0 {
			zero = append(zero, topic)
		}
	}
	return zero
}

func positiveCountTopics(topics []string, counts map[string]int) []string {
	var present []string
	for _, topic := range topics {
		if counts[topic] > 0 {
			present = append(present, topic)
		}
	}
	return present
}
