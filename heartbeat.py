import threading
import time

from enum import Enum, auto
from time import monotonic

from pymavlink import mavutil


class LinkState(Enum):

    DISCONNECTED = auto()
    CONNECTED = auto()
    TIMEOUT = auto()


class HeartbeatManager:

    def __init__(
        self,
        connection,
        dispatcher,
        tx_interval=1.0,
        timeout=3.0
    ):

        self._connection = connection
        self._dispatcher = dispatcher

        self._tx_interval = tx_interval
        self._timeout = timeout

        self._running = False
        self._thread = None

        self._lock = threading.RLock()

        self._state = LinkState.DISCONNECTED

        self._last_rx_heartbeat = None
        self._last_tx_heartbeat = None

        self._heartbeat_rx_count = 0
        self._heartbeat_tx_count = 0
        self._timeout_count = 0

    # =====================================================
    # START
    # =====================================================

    def start(self):

        with self._lock:

            if self._running:
                return

            self._running = True

        self._dispatcher.register(
            "HEARTBEAT",
            self._on_heartbeat
        )

        self._thread = threading.Thread(
            target=self._loop,
            daemon=True,
            name="HeartbeatManager"
        )

        self._thread.start()

    # =====================================================
    # STOP
    # =====================================================

    def stop(self):

        with self._lock:
            self._running = False

        if self._thread:
            self._thread.join(timeout=2.0)

        self._dispatcher.unregister(
            "HEARTBEAT",
            self._on_heartbeat
        )

    # =====================================================
    # PROPERTIES
    # =====================================================

    @property
    def connected(self):

        with self._lock:
            return self._state == LinkState.CONNECTED

    @property
    def state(self):

        with self._lock:
            return self._state

    @property
    def heartbeat_age(self):

        with self._lock:

            if self._last_rx_heartbeat is None:
                return None

            return monotonic() - self._last_rx_heartbeat

    # =====================================================
    # LOOP
    # =====================================================

    def _loop(self):

        next_tx = monotonic()

        while self._is_running():

            now = monotonic()

            # =============================================
            # SEND HEARTBEAT
            # =============================================

            if now >= next_tx:

                self._send_heartbeat()

                with self._lock:

                    self._last_tx_heartbeat = now
                    self._heartbeat_tx_count += 1

                self._dispatcher.dispatch(
                    "heartbeat_sent"
                )

                next_tx = now + self._tx_interval

            # =============================================
            # TIMEOUT CHECK
            # =============================================

            self._check_timeout(now)

            time.sleep(0.05)

    # =====================================================
    # SEND HEARTBEAT
    # =====================================================

    def _send_heartbeat(self):

        msg = self._connection.master.mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )

        self._connection.send(msg)

    # =====================================================
    # HEARTBEAT RX
    # =====================================================

    def _on_heartbeat(self, msg):

        now = monotonic()

        with self._lock:

            self._last_rx_heartbeat = now
            self._heartbeat_rx_count += 1

            previous_state = self._state

            self._state = LinkState.CONNECTED

        self._dispatcher.dispatch(
            "heartbeat_received",
            msg
        )
        
        # NOVO: Dispatch específico para watchdog e health monitor
        # Isso permite que o Health Monitor receba heartbeats diretamente
        self._dispatcher.dispatch(
            "heartbeat_watchdog",
            msg
        )

        if previous_state != LinkState.CONNECTED:

            self._dispatcher.dispatch(
                "link_connected"
            )

    # =====================================================
    # TIMEOUT
    # =====================================================

    def _check_timeout(self, now):

        with self._lock:

            if self._last_rx_heartbeat is None:
                return

            delta = now - self._last_rx_heartbeat

            if delta <= self._timeout:
                return

            if self._state == LinkState.TIMEOUT:
                return

            self._state = LinkState.TIMEOUT

            self._timeout_count += 1

        self._dispatcher.dispatch(
            "heartbeat_timeout"
        )

        self._dispatcher.dispatch(
            "link_disconnected"
        )

    # =====================================================
    # HELPERS
    # =====================================================

    def _is_running(self):

        with self._lock:
            return self._running