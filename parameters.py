"""
parameters.py

Gerenciamento de parâmetros MAVLink.
Suporta leitura, escrita e download completo de parâmetros.

Integrado com:
- Connection (envio/recebimento)
- Dispatcher (eventos assíncronos)
- TransactionManager (ACK handling)
"""

import threading
import time
from enum import Enum, auto
from typing import Optional, Dict, Any, Callable, Tuple
from dataclasses import dataclass

from pymavlink import mavutil


class ParamResult(Enum):
    """Resultados de operações com parâmetros."""
    SUCCESS = auto()
    TIMEOUT = auto()
    NOT_FOUND = auto()
    INVALID_VALUE = auto()
    WRITE_FAILED = auto()
    DISCONNECTED = auto()


@dataclass
class ParamValue:
    """Representa um parâmetro MAVLink com metadados."""
    name: str
    value: float
    param_type: int  # MAV_PARAM_TYPE
    param_index: int = -1
    param_count: int = -1

    def __repr__(self):
        return f"ParamValue(name='{self.name}', value={self.value})"


class ParameterManager:
    """
    Gerenciador completo de parâmetros MAVLink.
    
    Funcionalidades:
    - get_param: Leitura individual de parâmetro
    - set_param: Escrita individual de parâmetro
    - download_all: Download completo de todos os parâmetros
    - cache local thread-safe
    - busca por nome parcial
    """

    def __init__(self, connection, dispatcher, transaction):
        self._connection = connection
        self._dispatcher = dispatcher
        self._transaction = transaction

        # Cache thread-safe de parâmetros
        self._params: Dict[str, ParamValue] = {}
        self._lock = threading.RLock()

        # Controle de download assíncrono
        self._download_complete = threading.Event()
        self._download_in_progress = False
        self._param_count = 0
        self._params_received = 0

        # Registra listeners
        self._dispatcher.register("PARAM_VALUE", self._on_param_value)
        self._dispatcher.register("link_disconnected", self._on_disconnect)

    # ===================================================================
    # LEITURA DE PARÂMETRO
    # ===================================================================

    def get_param(self, param_name: str, timeout: float = 3.0) -> Tuple[ParamResult, Optional[ParamValue]]:
        """
        Lê um parâmetro específico do veículo.
        
        Args:
            param_name: Nome exato do parâmetro (case-sensitive)
            timeout: Tempo máximo de espera
            
        Returns:
            Tuple[ParamResult, Optional[ParamValue]]
        """
        # Verifica cache primeiro
        with self._lock:
            if param_name in self._params:
                return ParamResult.SUCCESS, self._params[param_name]

        # Prepara para receber resposta
        response_event = threading.Event()
        response_value = [None]

        def param_callback(msg):
            # Extrai nome do parâmetro da mensagem
            param_id = msg.param_id
            
            # Limpa caracteres nulos e converte para string
            if isinstance(param_id, bytes):
                param_id = param_id.decode('utf-8').strip('\x00').strip()
            else:
                param_id = str(param_id).strip('\x00').strip()
            
            if param_id == param_name:
                param = ParamValue(
                    name=param_id,
                    value=msg.param_value,
                    param_type=msg.param_type,
                    param_index=msg.param_index,
                    param_count=msg.param_count
                )
                response_value[0] = param
                
                # Atualiza cache também
                with self._lock:
                    self._params[param_id] = param
                
                response_event.set()

        # Registra callback temporário
        self._dispatcher.register("PARAM_VALUE", param_callback)

        try:
            # Envia requisição
            self._request_param_read(param_name)

            # Aguarda resposta
            if not response_event.wait(timeout=timeout):
                return ParamResult.TIMEOUT, None

            param = response_value[0]
            if param is None:
                return ParamResult.NOT_FOUND, None

            return ParamResult.SUCCESS, param

        finally:
            # Limpa callback temporário
            self._dispatcher.unregister("PARAM_VALUE", param_callback)

    def get_param_value(self, param_name: str, default: Any = None, timeout: float = 3.0) -> Optional[float]:
        """
        Lê apenas o valor de um parâmetro (conveniência).
        
        Args:
            param_name: Nome do parâmetro
            default: Valor padrão se falhar
            timeout: Timeout
            
        Returns:
            Valor do parâmetro ou default
        """
        result, param = self.get_param(param_name, timeout)
        if result == ParamResult.SUCCESS and param:
            return param.value
        return default

    # ===================================================================
    # ESCRITA DE PARÂMETRO
    # ===================================================================

    def set_param(self, param_name: str, value: float, 
                  param_type: int = mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                  timeout: float = 3.0) -> ParamResult:
        """
        Escreve um parâmetro no veículo.
        
        Args:
            param_name: Nome do parâmetro
            value: Novo valor
            param_type: Tipo MAVLink do parâmetro
            timeout: Timeout
            
        Returns:
            ParamResult
        """
        # Validação básica
        if not param_name or len(param_name) > 16:
            return ParamResult.INVALID_VALUE

        response_event = threading.Event()
        response_result = [ParamResult.TIMEOUT]
        response_value = [None]

        def param_callback(msg):
            # Extrai nome do parâmetro
            param_id = msg.param_id
            
            # Limpa caracteres nulos
            if isinstance(param_id, bytes):
                param_id = param_id.decode('utf-8').strip('\x00').strip()
            else:
                param_id = str(param_id).strip('\x00').strip()
            
            if param_id == param_name:
                # Verifica se o valor foi realmente alterado
                if abs(msg.param_value - value) < 1e-6:
                    response_result[0] = ParamResult.SUCCESS
                else:
                    response_result[0] = ParamResult.WRITE_FAILED
                
                response_value[0] = ParamValue(
                    name=param_id,
                    value=msg.param_value,
                    param_type=msg.param_type,
                    param_index=msg.param_index,
                    param_count=msg.param_count
                )
                
                # Atualiza cache
                with self._lock:
                    self._params[param_id] = response_value[0]
                
                response_event.set()

        # Registra callback
        self._dispatcher.register("PARAM_VALUE", param_callback)

        try:
            # Envia comando de escrita
            msg = self._connection.master.mav.param_set_encode(
                self._connection.master.target_system,
                self._connection.master.target_component,
                param_name.encode('utf-8'),
                float(value),
                param_type
            )
            self._connection.send(msg)

            # Aguarda confirmação
            if not response_event.wait(timeout=timeout):
                return ParamResult.TIMEOUT

            if response_result[0] == ParamResult.SUCCESS:
                # Já foi atualizado no cache dentro do callback
                pass

            return response_result[0]

        finally:
            self._dispatcher.unregister("PARAM_VALUE", param_callback)

    # ===================================================================
    # DOWNLOAD COMPLETO
    # ===================================================================

    def download_all(self, timeout: float = 30.0, 
                     progress_callback: Optional[Callable[[int, int, str], None]] = None) -> ParamResult:
        """
        Faz download de todos os parâmetros do veículo.
        
        Args:
            timeout: Timeout total (pode levar vários segundos)
            progress_callback: Callback opcional para progresso
                              callback(recebidos, total, nome_atual)
            
        Returns:
            ParamResult
        """
        if self._download_in_progress:
            print("[ParameterManager] Download já em andamento")
            return ParamResult.WRITE_FAILED

        self._download_in_progress = True
        self._download_complete.clear()
        self._params_received = 0
        self._param_count = 0

        # Parâmetros temporários durante download
        temp_params: Dict[str, ParamValue] = {}

        def param_handler(msg):
            # Extrai nome do parâmetro
            param_id = msg.param_id
            
            # Limpa caracteres nulos
            if isinstance(param_id, bytes):
                param_id = param_id.decode('utf-8').strip('\x00').strip()
            else:
                param_id = str(param_id).strip('\x00').strip()
            
            if not param_id:
                return
            
            param = ParamValue(
                name=param_id,
                value=msg.param_value,
                param_type=msg.param_type,
                param_index=msg.param_index,
                param_count=msg.param_count
            )

            temp_params[param_id] = param

            # Atualiza contagem total
            if msg.param_count > self._param_count:
                self._param_count = msg.param_count

            self._params_received = len(temp_params)

            # Callback de progresso
            if progress_callback:
                try:
                    progress_callback(
                        self._params_received, 
                        max(self._param_count, self._params_received),
                        param_id
                    )
                except Exception as e:
                    print(f"[ParameterManager] Erro no callback: {e}")

            # Verifica se completou
            if self._param_count > 0 and self._params_received >= self._param_count:
                self._download_complete.set()

        # Registra handler temporário
        self._dispatcher.register("PARAM_VALUE", param_handler)

        try:
            # Solicita lista de parâmetros
            msg = self._connection.master.mav.param_request_list_encode(
                self._connection.master.target_system,
                self._connection.master.target_component
            )
            self._connection.send(msg)

            print(f"[ParameterManager] Solicitando lista de parâmetros...")

            # Aguarda download completo
            start_time = time.time()

            while not self._download_complete.is_set():
                if time.time() - start_time > timeout:
                    print(f"[ParameterManager] Timeout após receber {len(temp_params)} parâmetros")
                    # Salva o que foi recebido até agora
                    with self._lock:
                        self._params.update(temp_params)
                    return ParamResult.TIMEOUT
                time.sleep(0.05)

            # Atualiza cache principal de forma thread-safe
            with self._lock:
                self._params.update(temp_params)

            print(f"[ParameterManager] Download completo: {len(temp_params)} parâmetros")
            return ParamResult.SUCCESS

        except Exception as e:
            print(f"[ParameterManager] Erro no download: {e}")
            return ParamResult.ERROR

        finally:
            self._dispatcher.unregister("PARAM_VALUE", param_handler)
            self._download_in_progress = False

    # ===================================================================
    # PARAM_REQUEST_READ (auxiliar)
    # ===================================================================

    def _request_param_read(self, param_name: str):
        """
        Solicita leitura de um parâmetro específico.
        """
        # Tenta usar param_request_read_encode
        msg = self._connection.master.mav.param_request_read_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            param_name.encode('utf-8'),
            -1  # param_index (-1 = usar nome)
        )
        self._connection.send(msg)

    # ===================================================================
    # HANDLERS DO DISPATCHER
    # ===================================================================

    def _on_param_value(self, msg):
        """
        Handler contínuo para PARAM_VALUE.
        Atualiza cache automaticamente em background.
        """
        # Extrai nome do parâmetro
        param_id = msg.param_id
        
        # Limpa caracteres nulos
        if isinstance(param_id, bytes):
            param_id = param_id.decode('utf-8').strip('\x00').strip()
        else:
            param_id = str(param_id).strip('\x00').strip()
        
        if not param_id:
            return
        
        param = ParamValue(
            name=param_id,
            value=msg.param_value,
            param_type=msg.param_type,
            param_index=msg.param_index,
            param_count=msg.param_count
        )

        with self._lock:
            self._params[param_id] = param

    def _on_disconnect(self):
        """
        Limpa estado em caso de desconexão.
        """
        self._download_in_progress = False
        self._download_complete.set()  # Libera threads bloqueadas

    # ===================================================================
    # MÉTODOS DE CONSULTA
    # ===================================================================

    def get_all_params(self) -> Dict[str, ParamValue]:
        """
        Retorna snapshot do cache de parâmetros.
        """
        with self._lock:
            return dict(self._params)

    def find_params(self, pattern: str) -> Dict[str, ParamValue]:
        """
        Busca parâmetros por nome parcial (case-insensitive).
        
        Args:
            pattern: Padrão de busca
            
        Returns:
            Dicionário com parâmetros encontrados
        """
        pattern_lower = pattern.lower()
        
        with self._lock:
            return {
                name: param 
                for name, param in self._params.items() 
                if pattern_lower in name.lower()
            }

    def list_params(self, filter_pattern: Optional[str] = None) -> list:
        """
        Lista parâmetros, opcionalmente filtrados.
        
        Args:
            filter_pattern: Padrão opcional para filtrar
            
        Returns:
            Lista de tuplas (nome, valor, tipo)
        """
        params = self.get_all_params()
        
        if filter_pattern:
            pattern_lower = filter_pattern.lower()
            params = {
                name: param 
                for name, param in params.items() 
                if pattern_lower in name.lower()
            }
        
        # Ordena por nome
        sorted_params = sorted(params.items(), key=lambda x: x[0].lower())
        
        return [
            (name, param.value, param.param_type)
            for name, param in sorted_params
        ]

    def has_param(self, param_name: str) -> bool:
        """
        Verifica se um parâmetro está no cache.
        """
        with self._lock:
            return param_name in self._params

    def param_count(self) -> int:
        """
        Retorna número de parâmetros no cache.
        """
        with self._lock:
            return len(self._params)

    # ===================================================================
    # COMPARAÇÃO E RESTORE
    # ===================================================================

    def compare_params(self, other_params: Dict[str, float]) -> Dict[str, tuple]:
        """
        Compara parâmetros atuais com outro conjunto.
        
        Args:
            other_params: Dicionário {nome: valor} para comparação
            
        Returns:
            Dicionário com diferenças: {nome: (valor_atual, valor_outro)}
        """
        differences = {}
        
        with self._lock:
            for name, other_value in other_params.items():
                if name in self._params:
                    current_value = self._params[name].value
                    if abs(current_value - other_value) > 1e-6:
                        differences[name] = (current_value, other_value)
                else:
                    differences[name] = (None, other_value)
        
        return differences

    def restore_params(self, params: Dict[str, float], 
                       timeout_per_param: float = 3.0) -> Dict[str, ParamResult]:
        """
        Restaura um conjunto de parâmetros para valores específicos.
        
        Args:
            params: Dicionário {nome: valor}
            timeout_per_param: Timeout por parâmetro
            
        Returns:
            Dicionário com resultados: {nome: ParamResult}
        """
        results = {}
        
        for name, value in params.items():
            print(f"  Restaurando {name} = {value}...")
            results[name] = self.set_param(name, value, timeout=timeout_per_param)
        
        return results

    # ===================================================================
    # EXPORTAÇÃO/IMPORTAÇÃO
    # ===================================================================

    def export_params(self, filepath: str) -> bool:
        """
        Exporta parâmetros para arquivo.
        Formato compatível com Mission Planner/QGC.
        
        Args:
            filepath: Caminho do arquivo
            
        Returns:
            bool indicando sucesso
        """
        try:
            with open(filepath, 'w') as f:
                f.write("# Parâmetros exportados pela biblioteca ppymavlink\n")
                f.write(f"# Data: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("# Formato: nome\tvalor\ttype\n")
                
                with self._lock:
                    # Ordena por nome para facilitar comparação
                    sorted_params = sorted(self._params.items(), key=lambda x: x[0].lower())
                    for name, param in sorted_params:
                        f.write(f"{name}\t{param.value}\t{param.param_type}\n")
            
            print(f"[ParameterManager] {len(self._params)} parâmetros exportados para {filepath}")
            return True
            
        except Exception as e:
            print(f"[ParameterManager] Erro ao exportar: {e}")
            return False

    def import_params(self, filepath: str) -> bool:
        """
        Importa parâmetros de arquivo (apenas cache, não escreve no veículo).
        
        Args:
            filepath: Caminho do arquivo
            
        Returns:
            bool indicando sucesso
        """
        try:
            count = 0
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    parts = line.split('\t')
                    if len(parts) >= 2:
                        name = parts[0].strip()
                        try:
                            value = float(parts[1])
                        except ValueError:
                            continue
                        
                        param_type = int(parts[2]) if len(parts) >= 3 else 0
                        
                        param = ParamValue(
                            name=name,
                            value=value,
                            param_type=param_type
                        )
                        
                        with self._lock:
                            self._params[name] = param
                        
                        count += 1
            
            print(f"[ParameterManager] {count} parâmetros importados de {filepath}")
            return True
            
        except Exception as e:
            print(f"[ParameterManager] Erro ao importar: {e}")
            return False

    # ===================================================================
    # UTILITÁRIOS
    # ===================================================================

    def get_param_info(self, param_name: str) -> Optional[Dict[str, Any]]:
        """
        Retorna informações detalhadas de um parâmetro.
        
        Args:
            param_name: Nome do parâmetro
            
        Returns:
            Dicionário com informações ou None
        """
        with self._lock:
            if param_name not in self._params:
                return None
            
            param = self._params[param_name]
            
            return {
                'name': param.name,
                'value': param.value,
                'type': param.param_type,
                'type_name': self._get_param_type_name(param.param_type),
                'index': param.param_index,
                'total_params': param.param_count
            }

    @staticmethod
    def _get_param_type_name(param_type: int) -> str:
        """
        Converte MAV_PARAM_TYPE para string legível.
        """
        type_names = {
            mavutil.mavlink.MAV_PARAM_TYPE_UINT8: 'uint8',
            mavutil.mavlink.MAV_PARAM_TYPE_INT8: 'int8',
            mavutil.mavlink.MAV_PARAM_TYPE_UINT16: 'uint16',
            mavutil.mavlink.MAV_PARAM_TYPE_INT16: 'int16',
            mavutil.mavlink.MAV_PARAM_TYPE_UINT32: 'uint32',
            mavutil.mavlink.MAV_PARAM_TYPE_INT32: 'int32',
            mavutil.mavlink.MAV_PARAM_TYPE_UINT64: 'uint64',
            mavutil.mavlink.MAV_PARAM_TYPE_INT64: 'int64',
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32: 'float',
            mavutil.mavlink.MAV_PARAM_TYPE_REAL64: 'double',
        }
        return type_names.get(param_type, f'unknown({param_type})')

    def print_param_summary(self):
        """
        Imprime resumo dos parâmetros em cache.
        """
        with self._lock:
            count = len(self._params)
            
            if count == 0:
                print("[ParameterManager] Nenhum parâmetro em cache")
                return
            
            # Agrupa por prefixo
            prefixes = {}
            for name in self._params.keys():
                prefix = name.split('_')[0] if '_' in name else name[:4]
                prefixes[prefix] = prefixes.get(prefix, 0) + 1
            
            print(f"\n[ParameterManager] {count} parâmetros em cache")
            print("  Principais grupos:")
            for prefix, cnt in sorted(prefixes.items(), key=lambda x: x[1], reverse=True)[:10]:
                print(f"    {prefix}_* : {cnt} parâmetros")

    # ===================================================================
    # CLEANUP
    # ===================================================================

    def stop(self):
        """
        Remove listeners e limpa recursos.
        """
        self._dispatcher.unregister("PARAM_VALUE", self._on_param_value)
        self._dispatcher.unregister("link_disconnected", self._on_disconnect)
        
        # Libera downloads pendentes
        self._download_in_progress = False
        self._download_complete.set()

    def clear_cache(self):
        """
        Limpa o cache de parâmetros.
        """
        with self._lock:
            self._params.clear()
            print("[ParameterManager] Cache limpo")

    def __repr__(self):
        with self._lock:
            return f"ParameterManager(cached={len(self._params)} params)"