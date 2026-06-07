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
