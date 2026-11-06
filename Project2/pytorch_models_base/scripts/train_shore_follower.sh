#!/bin/bash
# Run from this scripts/ folder, with the GPU venv active:
#   ./train_shore_follower.sh                      (uses ../../Recorded_data_shore)
#   ./train_shore_follower.sh /other/dataset/dir   (to train on another dataset)
# The output folder must NOT exist yet (the parser refuses to overwrite models):
# delete or rename ${BASE_DIR}/output before re-training.

BASE_DIR="${1:-../../Recorded_data_shore}"

# Shore follower: 3 classes (0 = turn with negative z, 1 = straight, 2 = turn with positive z)
python3 ../pytorch_models_base/train.py \
    --train_file ${BASE_DIR}/train/labels.txt \
    --train_root ${BASE_DIR}/train \
    --val_root ${BASE_DIR}/val \
    --val_file ${BASE_DIR}/val/labels.txt \
    --learning_rate 0.0001 \
    --weight_decay 0.001 \
    --batch_size 64 \
    --iter 30 \
    --dropout 0.5 \
    --classes 3 \
    --loss_func ce \
    --output ${BASE_DIR}/output
