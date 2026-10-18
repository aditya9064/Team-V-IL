#!/bin/bash
# After the extended Cosmos run AND tonight's Rosie hand-off (fresh models/cubes_v2.pt, cubes_v1v2.pt),
# score the new YOLO models on the 300-image set too, then rebuild its summary.
cd ~/adaptivevision
export PYTHONNOUSERSITE=1 HF_HOME=$PWD/hf YOLO_CONFIG_DIR=$PWD/ultralytics
until grep -q EXT_DONE baseline_ext.log 2>/dev/null; do sleep 300; done
for name in cubes_v2 cubes_v1v2; do
    until [ models/$name.pt -nt .ext2_start ]; do sleep 300; done
    sleep 60  # let the copy finish
    envs/cosmos/bin/python run_baseline.py fanuc_er4ia_sim baseline_set_ext --detector yolo \
        --weights models/$name.pt --name yolo_${name#cubes_} | tail -1
done
envs/cosmos/bin/python run_baseline.py fanuc_er4ia_sim baseline_set_ext --summary
echo EXT2_DONE
