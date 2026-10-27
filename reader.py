import threading
import time


class MAVLinkReader:
    """
    Responsável pelo loop contínuo de leitura MAVLink.
    """

    def __init__(self, connection, dispatcher):
        self.connection = connection
        self.dispatcher = dispatcher

        self._running = False
        self._thread = None

    def start(self):
        """
        Inicia a thread de leitura.
        """

        if self._running:
            return

        self._running = True

        self._thread = threading.Thread(
            target=self.loop,
            daemon=True,
            name="MAVLinkReaderThread"
        )

        self._thread.start()

    def stop(self):
        """
        Para a thread de leitura.
        """

        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=2)

    def loop(self):
        """
        Loop principal de leitura MAVLink.
        """

        while self._running:

            try:

                msg = self.connection.recv(
                    blocking=True,
                    timeout=1
                )

                if msg is None:
                    continue

                # Extrai o nome do tipo da mensagem (ex: "ATTITUDE", "HEARTBEAT")
                event_name = msg.get_type()
                
                # Dispatch com o nome do evento e a mensagem como argumento
                self.dispatcher.dispatch(event_name, msg)

            except Exception as e:
                print(f"[Reader] Erro: {e}")

                time.sleep(1)