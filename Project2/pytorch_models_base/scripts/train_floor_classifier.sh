#!/bin/bash

BASE_DIR="../../Recorded_data"
shift

# Traversable/Not
python3 ../pytorch_models_base/train.py \
    --train_file ${BASE_DIR}/train/labels.txt \
    --train_root ${BASE_DIR}/train \
    --val_root ${BASE_DIR}/val \
    --val_file ${BASE_DIR}/val/labels.txt \
    --learning_rate 0.001 \
    --weight_decay 0.001 \
    --batch_size 64 \
    --iter 50 \
    --dropout 0.2 \
    --classes 2 \
    --loss_func ce \
    --output ${BASE_DIR}/output \
    $*

