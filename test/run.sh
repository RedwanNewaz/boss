#!/usr/bin/env bash
EXE="../build/ompl_bow"
$EXE --config env2.yaml
python plot_boss2.py --csv_file output.csv --env_file env2.yaml