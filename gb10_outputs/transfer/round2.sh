#!/bin/bash
# Round 2: looser vis on the original renders, then white-balanced inputs.
cd ~/adaptivevision/transfer
OUT=/trial/sweep_out CONFIGS=vis05_med,vis03_med,vis05_high bash gb10_run.sh
IN=/trial/sweep_in_wb OUT=/trial/sweep_out_wb CONFIGS=edgevis_s80,vis05_med,vis03_med bash gb10_run.sh
echo ROUND2_DONE
