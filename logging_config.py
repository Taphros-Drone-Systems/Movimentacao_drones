"""
logging_config.py
Configuração centralizada de logging para a biblioteca ppymavlink.
"""

import logging
import sys
from typing import Optional


class ColoredFormatter(logging.Formatter):
    """Formatter com cores para console"""
    
    COLORS = {
        'DEBUG': '\033[36m',      # Cyan
        'INFO': '\033[32m',       # Green
        'WARNING': '\033[33m',    # Yellow
        'ERROR': '\033[31m',      # Red
        'CRITICAL': '\033[35m',   # Magenta
        'RESET': '\033[0m'
    }
    
    def format(self, record):
        levelname = record.levelname
        if levelname in self.COLORS:
            record.levelname = f"{self.COLORS[levelname]}{levelname}{self.COLORS['RESET']}"
        return super().format(record)


class LoggerManager:
    """Gerenciador centralizado de loggers"""
    
    _loggers = {}
    _initialized = False
    
    @classmethod
    def setup(
        cls,
        level: str = "INFO",
        log_file: Optional[str] = None,
        use_colors: bool = True,
        format_str: Optional[str] = None
    ):
        """
        Configura logging global.
        
        Args:
            level: Nível de log (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            log_file: Caminho para arquivo de log (opcional)
            use_colors: Usar cores no console
            format_str: Formato customizado (opcional)
        """
        if cls._initialized:
            return
        
        # Define formato padrão
        if format_str is None:
            format_str = '%(asctime)s | %(name)s | %(levelname)-8s | %(message)s'
        
        # Configura root logger
        root_logger = logging.getLogger()
        root_logger.setLevel(getattr(logging, level.upper()))
        
        # Remove handlers existentes
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)
        
        # Handler de console
        console_handler = logging.StreamHandler(sys.stdout)
        if use_colors:
            console_handler.setFormatter(ColoredFormatter(format_str))
        else:
            console_handler.setFormatter(logging.Formatter(format_str))
        root_logger.addHandler(console_handler)
        
        # Handler de arquivo (opcional)
        if log_file:
            file_handler = logging.FileHandler(log_file)
            file_handler.setFormatter(logging.Formatter(format_str))
            root_logger.addHandler(file_handler)
        
        cls._initialized = True
    
    @classmethod
    def get_logger(cls, name: str) -> logging.Logger:
        """Obtém logger específico"""
        if name not in cls._loggers:
            cls._loggers[name] = logging.getLogger(f"ppymavlink.{name}")
        return cls._loggers[name]
    
    @classmethod
    def set_level(cls, level: str):
        """Altera nível de log global"""
        logging.getLogger().setLevel(getattr(logging, level.upper()))


# Conveniência para acesso direto
def get_logger(name: str) -> logging.Logger:
    """Obtém logger configurado"""
    return LoggerManager.get_logger(name)