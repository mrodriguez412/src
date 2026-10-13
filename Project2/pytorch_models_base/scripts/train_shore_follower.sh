#!/bin/bash

BASE_DIR="$1"

# Shore follower
python3 train.py \
    --train_file ${BASE_DIR}/train/labels.txt \
    --train_root ${BASE_DIR}/train \
    --val_root ${BASE_DIR}/val \
    --val_file ${BASE_DIR}/val/labels.txt \
    --learning_rate 0.0001 \
    --weight_decay 0.001 \
    --batch_size 512 \
    --iter 25 \
    --dropout 0.1 \
    --classes 3 \
    --loss_func ce \
    --output ${BASE_DIR}/output
