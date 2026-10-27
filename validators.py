"""
validators.py
Validação de parâmetros e coordenadas para operações do veículo.
"""

import math
from typing import Tuple, Optional, List, Any
from dataclasses import dataclass
from enum import Enum


class ValidationSeverity(Enum):
    """Severidade da validação"""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


@dataclass
class ValidationResult:
    """Resultado de validação"""
    valid: bool
    message: str
    severity: ValidationSeverity = ValidationSeverity.ERROR
    
    def __bool__(self):
        return self.valid


class CoordinateValidator:
    """Validador de coordenadas geográficas"""
    
    # Limites
    MIN_LATITUDE = -90.0
    MAX_LATITUDE = 90.0
    MIN_LONGITUDE = -180.0
    MAX_LONGITUDE = 180.0
    
    @classmethod
    def validate_latitude(cls, lat: float) -> ValidationResult:
        """Valida latitude"""
        if not isinstance(lat, (int, float)):
            return ValidationResult(False, f"Latitude deve ser número, recebido: {type(lat)}")
        
        if math.isnan(lat):
            return ValidationResult(False, "Latitude não pode ser NaN")
        
        if math.isinf(lat):
            return ValidationResult(False, "Latitude não pode ser infinita")
        
        if lat < cls.MIN_LATITUDE or lat > cls.MAX_LATITUDE:
            return ValidationResult(
                False,
                f"Latitude fora do intervalo: {lat}° (deve estar entre {cls.MIN_LATITUDE}° e {cls.MAX_LATITUDE}°)"
            )
        
        return ValidationResult(True, "OK")
    
    @classmethod
    def validate_longitude(cls, lon: float) -> ValidationResult:
        """Valida longitude"""
        if not isinstance(lon, (int, float)):
            return ValidationResult(False, f"Longitude deve ser número, recebido: {type(lon)}")
        
        if math.isnan(lon):
            return ValidationResult(False, "Longitude não pode ser NaN")
        
        if math.isinf(lon):
            return ValidationResult(False, "Longitude não pode ser infinita")
        
        if lon < cls.MIN_LONGITUDE or lon > cls.MAX_LONGITUDE:
            return ValidationResult(
                False,
                f"Longitude fora do intervalo: {lon}° (deve estar entre {cls.MIN_LONGITUDE}° e {cls.MAX_LONGITUDE}°)"
            )
        
        return ValidationResult(True, "OK")
    
    @classmethod
    def validate_coordinates(cls, lat: float, lon: float) -> ValidationResult:
        """Valida par de coordenadas"""
        # Valida latitude
        lat_result = cls.validate_latitude(lat)
        if not lat_result:
            return lat_result
        
        # Valida longitude
        lon_result = cls.validate_longitude(lon)
        if not lon_result:
            return lon_result
        
        return ValidationResult(True, "Coordenadas válidas")


class AltitudeValidator:
    """Validador de altitude"""
    
    # Limites padrão (metros)
    DEFAULT_MIN_ALTITUDE = -10.0
    DEFAULT_MAX_ALTITUDE = 5000.0
    
    # Limites por tipo de operação
    OPERATION_LIMITS = {
        'takeoff': (0.5, 400.0),
        'land': (-1.0, 10.0),
        'waypoint': (0, 5000.0),
        'loiter': (0, 5000.0),
    }
    
    @classmethod
    def validate_altitude(
        cls,
        altitude: float,
        operation: Optional[str] = None,
        min_alt: Optional[float] = None,
        max_alt: Optional[float] = None
    ) -> ValidationResult:
        """
        Valida altitude.
        
        Args:
            altitude: Altitude em metros
            operation: Tipo de operação ('takeoff', 'land', 'waypoint', 'loiter')
            min_alt: Mínimo customizado (sobrescreve operation)
            max_alt: Máximo customizado (sobrescreve operation)
        """
        if not isinstance(altitude, (int, float)):
            return ValidationResult(False, f"Altitude deve ser número, recebido: {type(altitude)}")
        
        if math.isnan(altitude):
            return ValidationResult(False, "Altitude não pode ser NaN")
        
        if math.isinf(altitude):
            return ValidationResult(False, "Altitude não pode ser infinita")
        
        # Define limites
        if min_alt is not None and max_alt is not None:
            min_val = min_alt
            max_val = max_alt
        elif operation and operation in cls.OPERATION_LIMITS:
            min_val, max_val = cls.OPERATION_LIMITS[operation]
        else:
            min_val = cls.DEFAULT_MIN_ALTITUDE
            max_val = cls.DEFAULT_MAX_ALTITUDE
        
        # Valida
        if altitude < min_val:
            return ValidationResult(
                False,
                f"Altitude muito baixa: {altitude}m (mínimo para {operation or 'operação'}: {min_val}m)"
            )
        
        if altitude > max_val:
            return ValidationResult(
                False,
                f"Altitude muito alta: {altitude}m (máximo para {operation or 'operação'}: {max_val}m)"
            )
        
        # Aviso para altitudes muito baixas
        if operation == 'takeoff' and altitude < 2.0:
            return ValidationResult(
                True,
                f"Altitude de decolagem baixa: {altitude}m (recomendado: ≥ 2m)",
                severity=ValidationSeverity.WARNING
            )
        
        return ValidationResult(True, "Altitude válida")


class SpeedValidator:
    """Validador de velocidades"""
    
    MAX_GROUND_SPEED = 30.0
    MAX_VERTICAL_SPEED = 10.0
    MAX_YAW_RATE = 180.0
    
    @classmethod
    def validate_ground_speed(cls, speed: float) -> ValidationResult:
        """Valida velocidade de solo (m/s)"""
        if not isinstance(speed, (int, float)):
            return ValidationResult(False, f"Velocidade deve ser número, recebido: {type(speed)}")
        
        if speed < 0:
            return ValidationResult(False, f"Velocidade não pode ser negativa: {speed}m/s")
        
        if speed > cls.MAX_GROUND_SPEED:
            return ValidationResult(
                False,
                f"Velocidade muito alta: {speed}m/s (máximo: {cls.MAX_GROUND_SPEED}m/s)"
            )
        
        if speed > 20.0:
            return ValidationResult(
                True,
                f"Velocidade alta: {speed}m/s - verifique se é seguro",
                severity=ValidationSeverity.WARNING
            )
        
        return ValidationResult(True, "Velocidade válida")
    
    @classmethod
    def validate_vertical_speed(cls, speed: float) -> ValidationResult:
        """Valida velocidade vertical (m/s)"""
        if not isinstance(speed, (int, float)):
            return ValidationResult(False, f"Velocidade deve ser número, recebido: {type(speed)}")
        
        abs_speed = abs(speed)
        
        if abs_speed > cls.MAX_VERTICAL_SPEED:
            return ValidationResult(
                False,
                f"Velocidade vertical muito alta: {speed}m/s (máximo: ±{cls.MAX_VERTICAL_SPEED}m/s)"
            )
        
        if abs_speed > 5.0:
            return ValidationResult(
                True,
                f"Velocidade vertical alta: {speed}m/s - use com cuidado",
                severity=ValidationSeverity.WARNING
            )
        
        return ValidationResult(True, "Velocidade vertical válida")
    
    @classmethod
    def validate_yaw_rate(cls, rate: float) -> ValidationResult:
        """Valida taxa de yaw (graus/s)"""
        if not isinstance(rate, (int, float)):
            return ValidationResult(False, f"Taxa deve ser número, recebido: {type(rate)}")
        
        abs_rate = abs(rate)
        
        if abs_rate > cls.MAX_YAW_RATE:
            return ValidationResult(
                False,
                f"Taxa de yaw muito alta: {rate}°/s (máximo: ±{cls.MAX_YAW_RATE}°/s)"
            )
        
        if abs_rate > 90.0:
            return ValidationResult(
                True,
                f"Taxa de yaw alta: {rate}°/s - pode ser agressivo",
                severity=ValidationSeverity.WARNING
            )
        
        return ValidationResult(True, "Taxa de yaw válida")


class ParameterValidator:
    """Validador de parâmetros do veículo"""
    
    # Parâmetros críticos e seus limites
    CRITICAL_PARAMS = {
        'ARMING_CHECK': (0, 1),
        'FS_GCS_ENABLE': (0, 2),
        'BATT_MONITOR': (0, 4),
        'SYSID_THISMAV': (1, 255),
    }
    
    @classmethod
    def validate_param_value(cls, param_name: str, value: float) -> ValidationResult:
        """Valida valor de parâmetro específico"""
        param_upper = param_name.upper()
        
        if param_upper in cls.CRITICAL_PARAMS:
            min_val, max_val = cls.CRITICAL_PARAMS[param_upper]
            
            if value < min_val or value > max_val:
                return ValidationResult(
                    False,
                    f"Valor inválido para {param_name}: {value} (deve estar entre {min_val} e {max_val})"
                )
        
        return ValidationResult(True, "Valor válido")
    
    @classmethod
    def validate_param_name(cls, param_name: str) -> ValidationResult:
        """Valida nome do parâmetro"""
        if not param_name:
            return ValidationResult(False, "Nome do parâmetro não pode ser vazio")
        
        if len(param_name) > 16:
            return ValidationResult(
                False,
                f"Nome do parâmetro muito longo: {len(param_name)} caracteres (máximo: 16)"
            )
        
        # Verifica caracteres válidos (alfanuméricos e underscore)
        valid_chars = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_')
        invalid_chars = [c for c in param_name if c not in valid_chars]
        
        if invalid_chars:
            return ValidationResult(
                False,
                f"Nome do parâmetro contém caracteres inválidos: {invalid_chars}"
            )
        
        return ValidationResult(True, "Nome válido")


class MissionValidator:
    """Validador de missões"""
    
    MAX_MISSION_ITEMS = 32767
    MIN_WAYPOINT_DISTANCE = 0.1
    
    @classmethod
    def validate_mission_size(cls, num_items: int) -> ValidationResult:
        """Valida tamanho da missão"""
        if num_items <= 0:
            return ValidationResult(False, "Missão deve ter pelo menos 1 item")
        
        if num_items > cls.MAX_MISSION_ITEMS:
            return ValidationResult(
                False,
                f"Número de itens excede o limite: {num_items} (máximo: {cls.MAX_MISSION_ITEMS})"
            )
        
        if num_items > 100:
            return ValidationResult(
                True,
                f"Missão grande: {num_items} itens - upload pode demorar",
                severity=ValidationSeverity.WARNING
            )
        
        return ValidationResult(True, "Tamanho válido")
    
    @classmethod
    def validate_waypoint_sequence(cls, waypoints: List[Any]) -> ValidationResult:
        """Valida sequência de waypoints"""
        if not waypoints:
            return ValidationResult(False, "Lista de waypoints vazia")
        
        for i, wp in enumerate(waypoints):
            if not isinstance(wp, (tuple, list)) or len(wp) < 3:
                return ValidationResult(
                    False,
                    f"Waypoint {i} inválido: esperado (lat, lon, alt), recebido {wp}"
                )
            
            lat, lon, alt = wp[:3]
            
            # Valida coordenadas
            coord_result = CoordinateValidator.validate_coordinates(lat, lon)
            if not coord_result:
                return ValidationResult(False, f"Waypoint {i}: {coord_result.message}")
            
            # Valida altitude
            alt_result = AltitudeValidator.validate_altitude(alt, 'waypoint')
            if not alt_result:
                return ValidationResult(False, f"Waypoint {i}: {alt_result.message}")
        
        return ValidationResult(True, "Sequência válida")