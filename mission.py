"""
mission.py

Gerenciamento completo de missões MAVLink.
Suporta upload, download, clear e manipulação de waypoints.

Integrado com:
- Connection (envio/recebimento)
- Dispatcher (eventos assíncronos)
- TransactionManager (ACK handling)
"""

import threading
import time
from enum import Enum, auto
from typing import Optional, List, Callable, Tuple
from dataclasses import dataclass, field

from pymavlink import mavutil


class MissionResult(Enum):
    """Resultados de operações de missão."""
    SUCCESS = auto()
    TIMEOUT = auto()
    ERROR = auto()
    INVALID_SEQUENCE = auto()
    NO_MISSION = auto()
    CANCELLED = auto()
    DISCONNECTED = auto()


class MissionItem:
    """
    Representa um item de missão (waypoint/comando).
    """
    
    # Cache de nomes de comandos
    _command_names = {}
    
    def __init__(
        self,
        seq: int,
        command: int,
        frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current: int = 0,
        autocontinue: int = 1,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        mission_type: int = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    ):
        self.seq = seq
        self.command = command
        self.frame = frame
        self.current = current
        self.autocontinue = autocontinue
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.x = x
        self.y = y
        self.z = z
        self.mission_type = mission_type

    @staticmethod
    def _get_command_name(command_id: int) -> str:
        """
        Obtém nome do comando MAV_CMD de forma compatível.
        """
        if command_id in MissionItem._command_names:
            return MissionItem._command_names[command_id]
        
        cmd_name = f'CMD_{command_id}'
        
        try:
            # Tenta diferentes formas de acessar o enum
            mav_cmd_enum = mavutil.mavlink.enums.get('MAV_CMD', None)
            
            if mav_cmd_enum is not None:
                # Forma 1: dict com chaves numéricas
                if isinstance(mav_cmd_enum, dict):
                    entry = mav_cmd_enum.get(command_id)
                    if entry is not None:
                        if isinstance(entry, dict):
                            cmd_name = entry.get('name', cmd_name)
                        elif hasattr(entry, 'name'):
                            cmd_name = entry.name
                
                # Forma 2: objeto com items()
                elif hasattr(mav_cmd_enum, 'items'):
                    for name, value in mav_cmd_enum.items():
                        if value == command_id:
                            cmd_name = name
                            break
                
                # Forma 3: EnumEntry com atributos
                elif hasattr(mav_cmd_enum, 'name') and hasattr(mav_cmd_enum, 'value'):
                    # Procura no enum
                    for entry in mav_cmd_enum.__class__.__members__.values():
                        if entry.value == command_id:
                            cmd_name = entry.name
                            break
        except Exception:
            pass
        
        MissionItem._command_names[command_id] = cmd_name
        return cmd_name

    def to_mavlink(self, target_system: int, target_component: int):
        """
        Converte para mensagem MISSION_ITEM_INT.
        """
        return mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system,
            target_component,
            self.seq,
            self.frame,
            self.command,
            self.current,
            self.autocontinue,
            self.param1,
            self.param2,
            self.param3,
            self.param4,
            int(self.x * 1e7),  # Latitude
            int(self.y * 1e7),  # Longitude
            self.z,              # Altitude
            self.mission_type
        )

    @classmethod
    def from_mavlink(cls, msg):
        """
        Cria MissionItem a partir de mensagem MAVLink MISSION_ITEM ou MISSION_ITEM_INT.
        """
        # Converte coordenadas se for MISSION_ITEM_INT
        if hasattr(msg, 'x') and abs(msg.x) > 1.0:
            # Provavelmente já está em graus (MISSION_ITEM)
            x = msg.x
            y = msg.y
        elif hasattr(msg, 'x'):
            # MISSION_ITEM_INT - converte de 1e7
            x = msg.x / 1e7 if msg.x != 0 else 0.0
            y = msg.y / 1e7 if msg.y != 0 else 0.0
        else:
            x = 0.0
            y = 0.0
        
        return cls(
            seq=msg.seq,
            command=msg.command,
            frame=msg.frame if hasattr(msg, 'frame') else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current=msg.current,
            autocontinue=msg.autocontinue,
            param1=msg.param1,
            param2=msg.param2,
            param3=msg.param3,
            param4=msg.param4,
            x=x,
            y=y,
            z=msg.z,
            mission_type=getattr(msg, 'mission_type', 
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        )

    @classmethod
    def takeoff(cls, seq: int, altitude: float, 
                latitude: float = 0, longitude: float = 0):
        """
        Cria item de decolagem.
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            param1=0,      # Pitch mínimo
            param2=0,      # Unused
            param3=0,      # Unused
            param4=0,      # Yaw (0 = heading atual)
            x=latitude,    # Latitude
            y=longitude,   # Longitude
            z=altitude     # Altitude de decolagem
        )

    @classmethod
    def waypoint(cls, seq: int, latitude: float, longitude: float, 
                 altitude: float, delay: float = 0.0, 
                 acceptance_radius: float = 2.0):
        """
        Cria waypoint de navegação.
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            param1=delay,              # Delay em segundos
            param2=acceptance_radius,  # Raio de aceitação
            param3=0,                  # Passagem pelo waypoint
            param4=float('nan'),       # Yaw (NaN = manter)
            x=latitude,
            y=longitude,
            z=altitude
        )

    @classmethod
    def land(cls, seq: int, latitude: float = 0, longitude: float = 0, 
             altitude: float = 0):
        """
        Cria item de pouso.
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_NAV_LAND,
            x=latitude,
            y=longitude,
            z=altitude
        )

    @classmethod
    def rtl(cls, seq: int):
        """
        Cria comando Return to Launch.
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        )

    @classmethod
    def loiter(cls, seq: int, time_seconds: float, latitude: float = 0, 
               longitude: float = 0, altitude: float = 0, 
               radius: float = 10.0):
        """
        Cria comando de loiter (pairar por tempo).
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            param1=time_seconds,
            param3=radius,
            x=latitude,
            y=longitude,
            z=altitude
        )

    @classmethod
    def loiter_unlimited(cls, seq: int, latitude: float = 0, 
                        longitude: float = 0, altitude: float = 0, 
                        radius: float = 10.0):
        """
        Cria comando de loiter ilimitado (pairar indefinidamente).
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            param3=radius,
            x=latitude,
            y=longitude,
            z=altitude
        )

    @classmethod
    def do_set_servo(cls, seq: int, servo_num: int, pwm: int):
        """
        Comando para controlar servo.
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            param1=servo_num,
            param2=pwm
        )

    @classmethod
    def do_delay(cls, seq: int, seconds: float):
        """
        Comando para delay durante missão.
        """
        return cls(
            seq=seq,
            command=mavutil.mavlink.MAV_CMD_DO_DELAY,
            param1=seconds
        )

    def __repr__(self):
        cmd_name = self._get_command_name(self.command)
        return (f"MissionItem(seq={self.seq}, {cmd_name}, "
                f"x={self.x:.6f}, y={self.y:.6f}, z={self.z:.2f})")


class MissionManager:
    """
    Gerenciador completo de missões MAVLink.
    
    Funcionalidades:
    - upload: Envia missão para o veículo
    - download: Recebe missão do veículo
    - clear: Limpa missão do veículo
    - Manipulação local de waypoints
    - Cache thread-safe da missão atual
    """

    def __init__(self, connection, dispatcher, transaction):
        self._connection = connection
        self._dispatcher = dispatcher
        self._transaction = transaction

        # Cache da missão atual
        self._mission_items: List[MissionItem] = []
        self._lock = threading.RLock()

        # Controle de operações assíncronas
        self._operation_complete = threading.Event()
        self._operation_result = MissionResult.SUCCESS
        self._current_seq = 0
        self._mission_count = 0
        self._items_received = 0

        # Registra listeners
        self._dispatcher.register("MISSION_ACK", self._on_mission_ack)
        self._dispatcher.register("MISSION_ITEM", self._on_mission_item_generic)
        self._dispatcher.register("MISSION_ITEM_INT", self._on_mission_item_generic)
        self._dispatcher.register("MISSION_COUNT", self._on_mission_count)
        self._dispatcher.register("MISSION_REQUEST", self._on_mission_request)
        self._dispatcher.register("MISSION_REQUEST_INT", self._on_mission_request)
        self._dispatcher.register("link_disconnected", self._on_disconnect)

    # ===================================================================
    # UPLOAD DE MISSÃO
    # ===================================================================

    def upload(self, mission_items: Optional[List[MissionItem]] = None,
               timeout: float = 30.0, 
               progress_callback: Optional[Callable] = None) -> MissionResult:
        """
        Envia missão para o veículo.
        
        Args:
            mission_items: Lista de MissionItem (usa cache interno se None)
            timeout: Timeout total
            progress_callback: Callback(current, total)
            
        Returns:
            MissionResult
        """
        if mission_items is not None:
            with self._lock:
                self._mission_items = list(mission_items)

        with self._lock:
            items = list(self._mission_items)

        if not items:
            return MissionResult.NO_MISSION

        # Verifica sequência
        for i, item in enumerate(items):
            if item.seq != i:
                return MissionResult.INVALID_SEQUENCE

        self._operation_result = MissionResult.SUCCESS
        self._operation_complete.clear()
        self._mission_count = len(items)

        # Callback de progresso
        if progress_callback:
            progress_callback(0, len(items))

        try:
            # 1. Envia MISSION_COUNT
            msg = self._connection.master.mav.mission_count_encode(
                self._connection.master.target_system,
                self._connection.master.target_component,
                len(items),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            self._connection.send(msg)

            # 2. Aguarda conclusão (via MISSION_ACK)
            if not self._operation_complete.wait(timeout=timeout):
                return MissionResult.TIMEOUT

            if progress_callback:
                progress_callback(len(items), len(items))

            return self._operation_result

        except Exception as e:
            print(f"[MissionManager] Erro no upload: {e}")
            return MissionResult.ERROR

    def upload_item(self, item: MissionItem) -> MissionResult:
        """
        Adiciona um único item à missão atual e faz upload.
        """
        with self._lock:
            self._mission_items.append(item)
        return self.upload()

    def upload_waypoints(self, waypoints: List[Tuple[float, float, float]], 
                        takeoff_alt: float = 10.0,
                        include_takeoff: bool = True,
                        include_land: bool = True) -> MissionResult:
        """
        Converte lista de coordenadas em missão e faz upload.
        
        Args:
            waypoints: Lista de tuplas (lat, lon, alt)
            takeoff_alt: Altitude de decolagem
            include_takeoff: Inclui comando de decolagem
            include_land: Inclui comando de pouso
            
        Returns:
            MissionResult
        """
        items = []
        seq = 0

        if include_takeoff:
            items.append(MissionItem.takeoff(seq, takeoff_alt))
            seq += 1

        for lat, lon, alt in waypoints:
            items.append(MissionItem.waypoint(seq, lat, lon, alt))
            seq += 1

        if include_land:
            items.append(MissionItem.land(seq))

        with self._lock:
            self._mission_items = items

        return self.upload()

    # ===================================================================
    # DOWNLOAD DE MISSÃO
    # ===================================================================

    def download(self, timeout: float = 30.0,
                 progress_callback: Optional[Callable] = None) -> Tuple[MissionResult, List[MissionItem]]:
        """
        Baixa missão atual do veículo.
        
        Args:
            timeout: Timeout total
            progress_callback: Callback(current, total)
            
        Returns:
            Tuple[MissionResult, List[MissionItem]]
        """
        self._operation_result = MissionResult.SUCCESS
        self._operation_complete.clear()
        self._items_received = 0
        self._mission_count = 0

        temp_items = []

        # Handler temporário para receber itens
        def item_handler(msg):
            try:
                item = MissionItem.from_mavlink(msg)
                temp_items.append(item)
                self._items_received = len(temp_items)

                if progress_callback:
                    progress_callback(
                        self._items_received,
                        max(self._mission_count, self._items_received)
                    )
                
                # Solicita próximo item
                if self._mission_count > 0 and self._items_received < self._mission_count:
                    self._request_mission_item(self._items_received)
                elif self._mission_count > 0 and self._items_received >= self._mission_count:
                    # Todos os itens recebidos
                    self._operation_complete.set()
                    
            except Exception as e:
                print(f"[MissionManager] Erro ao processar item: {e}")

        # Registra handlers temporários
        self._dispatcher.register("MISSION_ITEM", item_handler)
        self._dispatcher.register("MISSION_ITEM_INT", item_handler)

        try:
            # 1. Solicita lista
            msg = self._connection.master.mav.mission_request_list_encode(
                self._connection.master.target_system,
                self._connection.master.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            self._connection.send(msg)

            # 2. Aguarda conclusão
            if not self._operation_complete.wait(timeout=timeout):
                return MissionResult.TIMEOUT, temp_items if temp_items else []

            # Ordena por sequência
            temp_items.sort(key=lambda x: x.seq)

            # Atualiza cache
            with self._lock:
                self._mission_items = temp_items

            return self._operation_result, temp_items

        except Exception as e:
            print(f"[MissionManager] Erro no download: {e}")
            return MissionResult.ERROR, temp_items if temp_items else []

        finally:
            # Remove handlers temporários
            self._dispatcher.unregister("MISSION_ITEM", item_handler)
            self._dispatcher.unregister("MISSION_ITEM_INT", item_handler)

    # ===================================================================
    # LIMPAR MISSÃO
    # ===================================================================

    def clear(self, timeout: float = 10.0) -> MissionResult:
        """
        Limpa a missão do veículo.
        
        Returns:
            MissionResult
        """
        self._operation_result = MissionResult.SUCCESS
        self._operation_complete.clear()

        try:
            msg = self._connection.master.mav.mission_clear_all_encode(
                self._connection.master.target_system,
                self._connection.master.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            self._connection.send(msg)

            if not self._operation_complete.wait(timeout=timeout):
                return MissionResult.TIMEOUT

            if self._operation_result == MissionResult.SUCCESS:
                with self._lock:
                    self._mission_items.clear()

            return self._operation_result

        except Exception as e:
            print(f"[MissionManager] Erro ao limpar missão: {e}")
            return MissionResult.ERROR

    # ===================================================================
    # MANIPULAÇÃO LOCAL
    # ===================================================================

    def add_item(self, item: MissionItem):
        """
        Adiciona item à missão local (cache).
        """
        with self._lock:
            item.seq = len(self._mission_items)
            self._mission_items.append(item)

    def remove_item(self, seq: int) -> bool:
        """
        Remove item por sequência.
        """
        with self._lock:
            if 0 <= seq < len(self._mission_items):
                self._mission_items.pop(seq)
                # Reenumera
                for i, item in enumerate(self._mission_items):
                    item.seq = i
                return True
            return False

    def insert_item(self, seq: int, item: MissionItem) -> bool:
        """
        Insere item em posição específica.
        """
        with self._lock:
            if 0 <= seq <= len(self._mission_items):
                item.seq = seq
                self._mission_items.insert(seq, item)
                # Reenumera a partir da posição
                for i in range(seq + 1, len(self._mission_items)):
                    self._mission_items[i].seq = i
                return True
            return False

    def get_item(self, seq: int) -> Optional[MissionItem]:
        """
        Retorna item por sequência.
        """
        with self._lock:
            if 0 <= seq < len(self._mission_items):
                return self._mission_items[seq]
            return None

    def get_mission(self) -> List[MissionItem]:
        """
        Retorna cópia da missão atual.
        """
        with self._lock:
            return list(self._mission_items)

    def set_mission(self, items: List[MissionItem]):
        """
        Define missão completa (apenas cache).
        """
        with self._lock:
            self._mission_items = list(items)
            # Reenumera
            for i, item in enumerate(self._mission_items):
                item.seq = i

    def mission_count(self) -> int:
        """
        Retorna número de itens na missão.
        """
        with self._lock:
            return len(self._mission_items)

    # ===================================================================
    # QUERIES
    # ===================================================================

    def get_current_item(self) -> Optional[MissionItem]:
        """
        Retorna o item atual da missão.
        """
        with self._lock:
            for item in self._mission_items:
                if item.current == 1:
                    return item
            return None

    def get_takeoff_item(self) -> Optional[MissionItem]:
        """
        Retorna o item de decolagem.
        """
        with self._lock:
            for item in self._mission_items:
                if item.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    return item
            return None

    def get_land_item(self) -> Optional[MissionItem]:
        """
        Retorna o item de pouso.
        """
        with self._lock:
            for item in self._mission_items:
                if item.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    return item
            return None

    # ===================================================================
    # HANDLERS DO DISPATCHER
    # ===================================================================

    def _on_mission_ack(self, msg):
        """
        Handler para MISSION_ACK.
        """
        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            self._operation_result = MissionResult.SUCCESS
        else:
            self._operation_result = MissionResult.ERROR
        
        self._operation_complete.set()

    def _on_mission_count(self, msg):
        """
        Handler para MISSION_COUNT.
        """
        if msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            self._mission_count = msg.count
            
            if msg.count == 0:
                # Missão vazia
                self._operation_complete.set()
            else:
                # Solicita primeiro item
                self._request_mission_item(0)

    def _on_mission_item_generic(self, msg):
        """
        Handler genérico para MISSION_ITEM e MISSION_ITEM_INT.
        Mantido para compatibilidade - handlers temporários no download
        têm precedência via dispatcher.
        """
        pass

    def _on_mission_request(self, msg):
        """
        Handler para MISSION_REQUEST durante upload.
        """
        if msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            seq = msg.seq
            
            with self._lock:
                if seq < len(self._mission_items):
                    item = self._mission_items[seq]
                    msg_out = item.to_mavlink(
                        self._connection.master.target_system,
                        self._connection.master.target_component
                    )
                    self._connection.send(msg_out)
                else:
                    # Sequência inválida
                    print(f"[MissionManager] Sequência inválida solicitada: {seq}")
                    self._operation_result = MissionResult.INVALID_SEQUENCE
                    self._operation_complete.set()

    def _on_disconnect(self):
        """
        Handler para desconexão.
        """
        self._operation_result = MissionResult.DISCONNECTED
        self._operation_complete.set()

    def _request_mission_item(self, seq: int):
        """
        Solicita um item de missão específico.
        Usa MISSION_REQUEST_INT para receber MISSION_ITEM_INT.
        """
        try:
            msg = self._connection.master.mav.mission_request_int_encode(
                self._connection.master.target_system,
                self._connection.master.target_component,
                seq,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            self._connection.send(msg)
        except AttributeError:
            # Fallback para mission_request_encode
            msg = self._connection.master.mav.mission_request_encode(
                self._connection.master.target_system,
                self._connection.master.target_component,
                seq,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )
            self._connection.send(msg)

    # ===================================================================
    # EXPORTAÇÃO/IMPORTAÇÃO
    # ===================================================================

    def export_mission(self, filepath: str) -> bool:
        """
        Exporta missão para arquivo (formato waypoint).
        """
        try:
            with open(filepath, 'w') as f:
                f.write("QGC WPL 110\n")  # Header
                
                with self._lock:
                    for i, item in enumerate(self._mission_items):
                        f.write(
                            f"{i}\t"
                            f"{item.current}\t"
                            f"{item.frame}\t"
                            f"{item.command}\t"
                            f"{item.param1}\t"
                            f"{item.param2}\t"
                            f"{item.param3}\t"
                            f"{item.param4}\t"
                            f"{item.x}\t"
                            f"{item.y}\t"
                            f"{item.z}\t"
                            f"{item.autocontinue}\n"
                        )
            
            print(f"[MissionManager] {len(self._mission_items)} itens exportados para {filepath}")
            return True
            
        except Exception as e:
            print(f"[MissionManager] Erro ao exportar: {e}")
            return False

    # ===================================================================
    # CLEANUP
    # ===================================================================

    def stop(self):
        """
        Remove listeners e limpa recursos.
        """
        self._dispatcher.unregister("MISSION_ACK", self._on_mission_ack)
        self._dispatcher.unregister("MISSION_ITEM", self._on_mission_item_generic)
        self._dispatcher.unregister("MISSION_ITEM_INT", self._on_mission_item_generic)
        self._dispatcher.unregister("MISSION_COUNT", self._on_mission_count)
        self._dispatcher.unregister("MISSION_REQUEST", self._on_mission_request)
        self._dispatcher.unregister("MISSION_REQUEST_INT", self._on_mission_request)
        self._dispatcher.unregister("link_disconnected", self._on_disconnect)
        
        self._operation_complete.set()

    def __repr__(self):
        with self._lock:
            return f"MissionManager(items={len(self._mission_items)})"