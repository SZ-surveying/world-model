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
	flag.StringVar(&output, "output", "", "output JSON path; defaults to <artifact-dir>/contract_audit.json")
	flag.Parse()
	if flag.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "usage: hover-contract-audit [--output path] <artifact-dir>")
		os.Exit(2)
	}
	artifactDir := flag.Arg(0)
	audit, err := tasks.BuildHoverContractAudit(artifactDir)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	if output == "" {
		output = filepath.Join(artifactDir, "contract_audit.json")
	}
	data, err := json.MarshalIndent(audit, "", "  ")
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	if err := os.WriteFile(output, append(data, '\n'), 0o644); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	fmt.Println(output)
}
