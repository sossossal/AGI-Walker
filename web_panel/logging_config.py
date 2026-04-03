import logging
import sys
import os
from pythonjsonlogger import jsonlogger

def setup_logging():
    """工业级结构化日志配置。"""
    log_level = os.getenv("AGI_WALKER_LOG_LEVEL", "INFO")
    log_format = os.getenv("AGI_WALKER_LOG_FORMAT", "JSON") # TEXT or JSON
    
    root_logger = logging.getLogger()
    
    # 移除现有 handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
        
    handler = logging.StreamHandler(sys.stdout)
    
    if log_format.upper() == "JSON":
        formatter = jsonlogger.JsonFormatter(
            '%(timestamp)s %(level)s %(name)s %(message)s %(funcName)s %(lineno)d',
            timestamp=True
        )
    else:
        formatter = logging.Formatter(
            '[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s'
        )
        
    handler.setFormatter(formatter)
    root_logger.addHandler(handler)
    root_logger.setLevel(log_level)
    
    # 抑制三方库的冗余日志
    logging.getLogger("uvicorn.access").setLevel("WARNING")
    logging.getLogger("sqlalchemy.engine").setLevel("WARNING")
