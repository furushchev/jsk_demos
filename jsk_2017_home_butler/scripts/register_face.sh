#!/bin/bash

IMAGE=interaction/apply_mask/output
FACES=interaction/face_detection/faces
TRAIN=interaction/face_recognition/train

rosrun opencv_apps face_recognition_trainer.py image:=$IMAGE faces:=$FACES train:=$TRAIN
