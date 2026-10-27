from pymavlink import mavutil


class MAVLinkConnection:
    """
    Camada base de conexão MAVLink.
    Responsável por conectar, desconectar,
    enviar e receber mensagens MAVLink.
    """

    def __init__(self, connection_string: str, baudrate: int = 57600):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None

    def connect(self):
        """
        Cria conexão MAVLink.
        """

        self.master = mavutil.mavlink_connection(
            self.connection_string,
            baud=self.baudrate
        )

        print(f"[INFO] Conectado: {self.connection_string}")

    def disconnect(self):
        """
        Fecha conexão MAVLink.
        """

        if self.master:
            self.master.close()
            self.master = None

            print("[INFO] Conexão fechada")

    def wait_heartbeat(self, timeout: int = 30):
        """
        Aguarda HEARTBEAT do autopilot.
        """

        if not self.master:
            raise ConnectionError("Conexão MAVLink não inicializada")

        print("[INFO] Esperando HEARTBEAT...")

        self.master.wait_heartbeat(timeout=timeout)

        print(
            f"[INFO] HEARTBEAT recebido "
            f"(sistema {self.master.target_system}, "
            f"componente {self.master.target_component})"
        )

    def recv(self, blocking: bool = True, timeout: float = None):
        """
        Recebe próxima mensagem MAVLink.
        """

        if not self.master:
            raise ConnectionError("Conexão MAVLink não inicializada")

        msg = self.master.recv_match(
            blocking=blocking,
            timeout=timeout
        )

        return msg

    def send(self, message):
        """
        Envia mensagem MAVLink.
        """

        if not self.master:
            raise ConnectionError("Conexão MAVLink não inicializada")

        self.master.mav.send(message)