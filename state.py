
"""
state.py

Camada de armazenamento thread-safe do estado do veículo.

Responsabilidades:
- armazenar estado interno do veículo
- fornecer acesso thread-safe
- snapshots seguros
- desacoplamento total do MAVLink
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from threading import RLock
from time import time
from typing import Optional


# =========================================================
# DOMAIN MODELS
# =========================================================


@dataclass
class AttitudeState:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    roll_speed: float = 0.0
    pitch_speed: float = 0.0
    yaw_speed: float = 0.0

    last_update: float = 0.0


@dataclass
class PositionState:
    latitude: Optional[float] = None
    longitude: Optional[float] = None

    relative_altitude: Optional[float] = None
    absolute_altitude: Optional[float] = None

    vx: Optional[float] = None
    vy: Optional[float] = None
    vz: Optional[float] = None

    heading: Optional[float] = None

    last_update: float = 0.0


@dataclass
class BatteryState:
    voltage: Optional[float] = None
    current: Optional[float] = None
    remaining: Optional[int] = None

    last_update: float = 0.0


@dataclass
class GPSState:
    satellites_visible: Optional[int] = None
    fix_type: Optional[int] = None

    last_update: float = 0.0


@dataclass
class SystemState:
    armed: bool = False

    flight_mode: str = "UNKNOWN"

    system_status: Optional[int] = None

    last_update: float = 0.0


@dataclass
class VehicleState:
    attitude: AttitudeState = field(default_factory=AttitudeState)

    position: PositionState = field(default_factory=PositionState)

    battery: BatteryState = field(default_factory=BatteryState)

    gps: GPSState = field(default_factory=GPSState)

    system: SystemState = field(default_factory=SystemState)


# =========================================================
# STATE MANAGER
# =========================================================


class StateManager:

    def __init__(self):
        self._lock = RLock()

        self._state = VehicleState()

    # =====================================================
    # ATTITUDE
    # =====================================================

    def update_attitude(
        self,
        roll,
        pitch,
        yaw,
        roll_speed,
        pitch_speed,
        yaw_speed
    ):
        with self._lock:

            state = self._state.attitude

            state.roll = roll
            state.pitch = pitch
            state.yaw = yaw

            state.roll_speed = roll_speed
            state.pitch_speed = pitch_speed
            state.yaw_speed = yaw_speed

            state.last_update = time()

    # =====================================================
    # POSITION
    # =====================================================

    def update_position(
        self,
        latitude,
        longitude,
        relative_altitude,
        absolute_altitude,
        vx,
        vy,
        vz,
        heading
    ):
        with self._lock:

            state = self._state.position

            state.latitude = latitude
            state.longitude = longitude

            state.relative_altitude = relative_altitude
            state.absolute_altitude = absolute_altitude

            state.vx = vx
            state.vy = vy
            state.vz = vz

            state.heading = heading

            state.last_update = time()

    # =====================================================
    # BATTERY
    # =====================================================

    def update_battery(
        self,
        voltage,
        current,
        remaining
    ):
        with self._lock:

            state = self._state.battery

            state.voltage = voltage
            state.current = current
            state.remaining = remaining

            state.last_update = time()

    # =====================================================
    # GPS
    # =====================================================

    def update_gps(
        self,
        satellites_visible,
        fix_type
    ):
        with self._lock:

            state = self._state.gps

            state.satellites_visible = satellites_visible
            state.fix_type = fix_type

            state.last_update = time()

    # =====================================================
    # SYSTEM
    # =====================================================

    def update_system(
        self,
        armed,
        flight_mode,
        system_status
    ):
        with self._lock:

            state = self._state.system

            state.armed = armed
            state.flight_mode = flight_mode
            state.system_status = system_status

            state.last_update = time()

    # =====================================================
    # SNAPSHOT
    # =====================================================

    def get_state(self) -> VehicleState:
        with self._lock:
            return deepcopy(self._state)

    # =====================================================
    # GETTERS
    # =====================================================

    @property
    def attitude(self):
        with self._lock:
            return deepcopy(self._state.attitude)

    @property
    def position(self):
        with self._lock:
            return deepcopy(self._state.position)

    @property
    def battery(self):
        with self._lock:
            return deepcopy(self._state.battery)

    @property
    def gps(self):
        with self._lock:
            return deepcopy(self._state.gps)

    @property
    def system(self):
        with self._lock:
            return deepcopy(self._state.system)

    # =====================================================
    # HELPERS
    # =====================================================

    def is_armed(self):
        with self._lock:
            return self._state.system.armed

    def flight_mode(self):
        with self._lock:
            return self._state.system.flight_mode

    def has_gps_fix(self):
        with self._lock:
            return (
                self._state.gps.fix_type is not None
                and self._state.gps.fix_type >= 3
            )

    # =====================================================
    # DEBUG
    # =====================================================

    def __repr__(self):
        return (
            f"StateManager("
            f"armed={self._state.system.armed}, "
            f"mode='{self._state.system.flight_mode}'"
            f")"
        )