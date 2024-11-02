#!/usr/bin/env bash
EXE="../build/ompl_bow"
$EXE --config cbo_param.yaml
python plot_boss2.py --csv_file output.csv