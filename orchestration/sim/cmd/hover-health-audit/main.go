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
	flag.StringVar(&output, "output", "", "output JSON path; defaults to <artifact-dir>/hover_health_summary.json for one run, stdout for cohorts")
	flag.Parse()
	if flag.NArg() < 1 {
		fmt.Fprintln(os.Stderr, "usage: hover-health-audit [--output path] <artifact-dir> [<artifact-dir>...]")
		os.Exit(2)
	}

	var payload any
	if flag.NArg() == 1 {
		artifactDir := flag.Arg(0)
		audit, err := tasks.BuildHoverHealthAudit(artifactDir)
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		payload = audit
		if output == "" {
			output = filepath.Join(artifactDir, "hover_health_summary.json")
		}
	} else {
		cohort, err := tasks.BuildHoverHealthCohort(flag.Args())
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		payload = cohort
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
