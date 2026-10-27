

from math import degrees

from pymavlink import mavutil


class Telemetry:

    """
    Camada de interpretação MAVLink.

    Responsabilidades:
    - receber mensagens MAVLink
    - converter unidades
    - atualizar StateManager
    - desacoplar MAVLink do estado interno
    """

    def __init__(self, dispatcher, state):

        self.dispatcher = dispatcher
        self.state = state

        self._register_handlers()

    # =====================================================
    # REGISTRO
    # =====================================================

    def _register_handlers(self):

        self.dispatcher.register(
            "ATTITUDE",
            self._handle_attitude
        )

        self.dispatcher.register(
            "GLOBAL_POSITION_INT",
            self._handle_global_position
        )

        self.dispatcher.register(
            "SYS_STATUS",
            self._handle_sys_status
        )

        self.dispatcher.register(
            "GPS_RAW_INT",
            self._handle_gps_raw
        )

        self.dispatcher.register(
            "HEARTBEAT",
            self._handle_heartbeat
        )

    # =====================================================
    # CONVERTERS
    # =====================================================

    @staticmethod
    def _rad_to_deg(value):
        return degrees(value)

    @staticmethod
    def _gps(value):
        return value / 1e7

    @staticmethod
    def _mm_to_m(value):
        return value / 1000.0

    @staticmethod
    def _cm_to_m(value):
        return value / 100.0

    @staticmethod
    def _mv_to_v(value):
        return value / 1000.0

    @staticmethod
    def _cA_to_A(value):
        return value / 100.0

    # =====================================================
    # ATTITUDE
    # =====================================================

    def _handle_attitude(self, msg):

        self.state.update_attitude(
            roll=self._rad_to_deg(msg.roll),
            pitch=self._rad_to_deg(msg.pitch),
            yaw=self._rad_to_deg(msg.yaw),

            roll_speed=self._rad_to_deg(msg.rollspeed),
            pitch_speed=self._rad_to_deg(msg.pitchspeed),
            yaw_speed=self._rad_to_deg(msg.yawspeed)
        )

    # =====================================================
    # GLOBAL POSITION
    # =====================================================

    def _handle_global_position(self, msg):

        self.state.update_position(
            latitude=self._gps(msg.lat),
            longitude=self._gps(msg.lon),

            relative_altitude=self._mm_to_m(
                msg.relative_alt
            ),

            absolute_altitude=self._mm_to_m(
                msg.alt
            ),

            vx=self._cm_to_m(msg.vx),
            vy=self._cm_to_m(msg.vy),
            vz=self._cm_to_m(msg.vz),

            heading=msg.hdg / 100.0
        )

    # =====================================================
    # SYS STATUS
    # =====================================================

    def _handle_sys_status(self, msg):

        voltage = None
        current = None
        remaining = None

        if msg.voltage_battery != -1:
            voltage = self._mv_to_v(
                msg.voltage_battery
            )

        if msg.current_battery != -1:
            current = self._cA_to_A(
                msg.current_battery
            )

        if msg.battery_remaining != -1:
            remaining = msg.battery_remaining

        self.state.update_battery(
            voltage=voltage,
            current=current,
            remaining=remaining
        )

    # =====================================================
    # GPS RAW
    # =====================================================

    def _handle_gps_raw(self, msg):

        self.state.update_gps(
            satellites_visible=msg.satellites_visible,
            fix_type=msg.fix_type
        )

    # =====================================================
    # HEARTBEAT
    # =====================================================

    def _handle_heartbeat(self, msg):

        armed = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        flight_mode = mavutil.mode_string_v10(msg)

        self.state.update_system(
            armed=armed,
            flight_mode=flight_mode,
            system_status=msg.system_status
        )

    # =====================================================
    # STOP
    # =====================================================

    def stop(self):

        self.dispatcher.unregister(
            "ATTITUDE",
            self._handle_attitude
        )

        self.dispatcher.unregister(
            "GLOBAL_POSITION_INT",
            self._handle_global_position
        )

        self.dispatcher.unregister(
            "SYS_STATUS",
            self._handle_sys_status
        )

        self.dispatcher.unregister(
            "GPS_RAW_INT",
            self._handle_gps_raw
        )

        self.dispatcher.unregister(
            "HEARTBEAT",
            self._handle_heartbeat
        )