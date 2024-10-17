from loguru import logger
import sys
import os



def set_log(log_level, log_dir):
    # 确保日志目录存在
    os.makedirs(log_dir, exist_ok=True)
    log_level = log_level.upper()
    
    # 配置 loguru
    config = {
        "handlers": [
            {"sink": sys.stdout, "format": "{thread.name} | <green>{time:YYYY-MM-DD at HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>"},
            {"sink": os.path.join(log_dir, "doris_trigger.log"), "rotation": "10 MB", "compression": "zip", "format": "{thread.name} | {time:YYYY-MM-DD at HH:mm:ss} | {level: <8} | {name}:{function}:{line} - {message}"}
        ],
        "extra": {"user": "doris_trigger"}
    }

    # 移除默认的 logger
    logger.remove()

    # 应用新的配置
    logger.configure(**config)

    # 设置日志级别（可以根据需要调整）
    logger.level(log_level)
