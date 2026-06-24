package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"

	"navlab/orchestration-sim/internal/tasks"
)

func main() {
	var output string
	flag.StringVar(&output, "output", "", "output JSON path; defaults to <artifact-dir>/initialization_audit.json for one run, stdout for multiple")
	flag.Parse()
	if flag.NArg() < 1 {
		fmt.Fprintln(os.Stderr, "usage: hover-init-audit [--output path] <artifact-dir> [<artifact-dir>...]")
		os.Exit(2)
	}

	audits := make([]map[string]any, 0, flag.NArg())
	for _, artifactDir := range flag.Args() {
		audit, err := tasks.BuildHoverInitializationAudit(artifactDir)
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		audit["artifact_dir"] = artifactDir
		audits = append(audits, audit)
		if flag.NArg() == 1 && output == "" {
			output = filepath.Join(artifactDir, "initialization_audit.json")
		}
	}

	var payload any
	if len(audits) == 1 {
		payload = audits[0]
	} else {
		payload = map[string]any{
			"schema":                    "navlab.hover_initialization_audit.comparison.v1",
			"diagnostic_only":           true,
			"runtime_control_unchanged": true,
			"runs":                      audits,
		}
	}
	data, err := json.MarshalIndent(payload, "", "  ")
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	data = append(data, '\n')
	if output == "" {
		if _, err := os.Stdout.Write(data); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	if err := os.WriteFile(output, data, 0o644); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	fmt.Println(output)
}
