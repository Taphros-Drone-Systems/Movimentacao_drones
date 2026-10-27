"""
transaction.py

Gerenciador de transacoes MAVLink.
Suporta request/response matching, timeout e retries.
"""

import threading
import time
from enum import Enum, auto
from typing import Optional, Callable, Dict


class TransactionStatus(Enum):
    PENDING = auto()
    SUCCESS = auto()
    TIMEOUT = auto()
    FAILED = auto()


class Transaction:
    """
    Representa uma transacao MAVLink pendente.
    """

    def __init__(
        self,
        command_id: int,
        expected_type: str,
        timeout: float = 3.0,
        retries: int = 3,
        callback: Optional[Callable] = None
    ):
        self.command_id = command_id
        self.expected_type = expected_type
        self.timeout = timeout
        self.retries = retries

        self.status = TransactionStatus.PENDING
        self.response = None
        self.callback = callback

        self._created_at = time.time()
        self._attempts = 0
        self._lock = threading.Lock()


class TransactionManager:
    """
    Gerencia transacoes MAVLink assincronas.
    Trabalha com o Dispatcher para receber respostas.
    """

    def __init__(self, dispatcher):
        self._dispatcher = dispatcher
        self._transactions: Dict[int, Transaction] = {}
        self._lock = threading.RLock()
        self._running = False
        self._monitor_thread = None

        # Registra listeners
        self._dispatcher.register("COMMAND_ACK", self._on_command_ack)

    def start(self):
        """
        Inicia thread de monitoramento de timeouts.
        """
        if self._running:
            return

        self._running = True
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="TransactionMonitor"
        )
        self._monitor_thread.start()

    def stop(self):
        """
        Para monitoramento e limpa listeners.
        """
        self._running = False

        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)

        self._dispatcher.unregister("COMMAND_ACK", self._on_command_ack)

    def create_transaction(
        self,
        command_id: int,
        expected_type: str = "COMMAND_ACK",
        timeout: float = 3.0,
        retries: int = 3,
        callback: Optional[Callable] = None
    ) -> Transaction:
        """
        Cria e registra uma nova transacao.
        """
        transaction = Transaction(
            command_id=command_id,
            expected_type=expected_type,
            timeout=timeout,
            retries=retries,
            callback=callback
        )

        with self._lock:
            self._transactions[command_id] = transaction

        return transaction

    def wait(self, transaction: Transaction) -> TransactionStatus:
        """
        Aguarda conclusao da transacao (bloqueante).
        """
        deadline = time.time() + transaction.timeout

        while time.time() < deadline:
            with transaction._lock:
                if transaction.status != TransactionStatus.PENDING:
                    return transaction.status

            time.sleep(0.01)

        with transaction._lock:
            if transaction.status == TransactionStatus.PENDING:
                transaction.status = TransactionStatus.TIMEOUT

        return TransactionStatus.TIMEOUT

    def _on_command_ack(self, msg):
        """
        Callback do dispatcher para COMMAND_ACK.
        """
        with self._lock:
            transaction = self._transactions.get(msg.command)

        if transaction is None:
            return

        with transaction._lock:
            if msg.result == 0:  # MAV_RESULT_ACCEPTED
                transaction.status = TransactionStatus.SUCCESS
            else:
                transaction.status = TransactionStatus.FAILED

            transaction.response = msg

        if transaction.callback:
            transaction.callback(transaction)

        # Remove transacao concluida
        with self._lock:
            self._transactions.pop(msg.command, None)

    def _monitor_loop(self):
        """
        Monitora timeouts das transacoes pendentes.
        """
        while self._running:
            now = time.time()

            with self._lock:
                expired = []

                for cmd_id, transaction in self._transactions.items():
                    with transaction._lock:
                        if transaction.status != TransactionStatus.PENDING:
                            expired.append(cmd_id)
                            continue

                        if (now - transaction._created_at) > transaction.timeout:
                            transaction.status = TransactionStatus.TIMEOUT
                            expired.append(cmd_id)

                            if transaction.callback:
                                transaction.callback(transaction)

                for cmd_id in expired:
                    self._transactions.pop(cmd_id, None)

            time.sleep(0.1)

    def cleanup(self):
        """
        Limpa todas as transacoes pendentes.
        """
        with self._lock:
            for transaction in self._transactions.values():
                with transaction._lock:
                    if transaction.status == TransactionStatus.PENDING:
                        transaction.status = TransactionStatus.TIMEOUT

            self._transactions.clear()