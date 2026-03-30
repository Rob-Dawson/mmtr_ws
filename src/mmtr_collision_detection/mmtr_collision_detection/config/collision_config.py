#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class DirectionWeights:
    yaw              :float
    pitch            :float
    axis             :float
    dominant_axis    :float

@dataclass
class Thresholds:
    yaw: float
    pitch: float
    ix: float
    iy: float

@dataclass
class Weights:
    front_rear: DirectionWeights
    left_right: DirectionWeights

@dataclass
class Config:
    thresholds: Thresholds
    weights: Weights

@dataclass
class ImuSample:
    t_ns: int
    jerk_mag: float
    jerk_x: float
    jerk_y: float
    jerk_z: float
