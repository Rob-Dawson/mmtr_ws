#!/usr/bin/env python3
from dataclasses import dataclass, field

@dataclass
class DirectionWeights:
    yaw              :float = 0.0
    pitch            :float = 0.0
    axis             :float = 0.0
    dominant_axis    :float = 0.0

@dataclass
class Thresholds:
    yaw: float      = 0.0
    pitch: float    = 0.0
    ix: float       = 0.0
    iy: float       = 0.0



@dataclass
class Weights:
    front_rear: DirectionWeights = field(default_factory=DirectionWeights)
    left_right: DirectionWeights = field(default_factory=DirectionWeights)

@dataclass
class Config:
    thresholds: Thresholds  = field(default_factory=Thresholds)
    weights: Weights        = field(default_factory=Weights)

@dataclass
class ImuSample:
    t_ns: int       = 0
    jerk_mag: float = 0.0
    jerk_x: float   = 0.0
    jerk_y: float   = 0.0
    jerk_z: float   = 0.0
