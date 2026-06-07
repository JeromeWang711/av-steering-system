# Autonomous Vehicle Steering System

Real-time cone detection and steering control for a self-driving miniature vehicle.

## Overview
Developed as part of DIT637 at University of Gothenburg.
The vehicle framework was provided by the course (developed by Prof. Dr. Christian Berger).
This repository contains only my personal implementation of the core detection and control algorithm.

Framework reference: https://github.com/chalmers-revere/working-with-rec-files/tree/master/cpp-opencv

## How it works
- Captures live video via shared memory IPC
- Detects blue (left) and yellow (right) road cones using HSV color segmentation
- Applies morphological closing to reduce noise
- Calculates steering angle using proportional control with exponential moving average smoothing
- Handles single-cone scenarios using adaptive road width estimation

## Technologies
C++, OpenCV, Linux, Shared Memory IPC

## Testing
Algorithm output was logged and evaluated against ground truth steering angles from pre-recorded driving data.

Run the evaluation script in the same directory as the log file:
```bash
python3 evaluate.py
```

Passing threshold: ≥35% accuracy within ±0.09 tolerance on non-zero ground truth frames.
