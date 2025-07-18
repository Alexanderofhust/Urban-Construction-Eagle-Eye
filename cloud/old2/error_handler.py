# Urban Construction Eagle Eye - 错误处理和日志配置

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional
import traceback
from functools import wraps

# 创建日志目录
LOG_DIR = Path("./logs")
LOG_DIR.mkdir(exist_ok=True)

# 日志格式
LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# 配置根日志器
logging.basicConfig(
    level=logging.INFO,
    format=LOG_FORMAT,
    datefmt=DATE_FORMAT,
    handlers=[
        logging.FileHandler(LOG_DIR / f"eagle_eye_{datetime.now().strftime('%Y%m%d')}.log", encoding='utf-8'),
        logging.StreamHandler(sys.stdout)
    ]
)

# 获取应用日志器
logger = logging.getLogger('eagle_eye')

class EagleEyeError(Exception):
    """基础异常类"""
    
    def __init__(self, message: str, error_code: str = None, details: Dict[str, Any] = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code or "UNKNOWN_ERROR"
        self.details = details or {}
        self.timestamp = datetime.now()
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'error_code': self.error_code,
            'message': self.message,
            'details': self.details,
            'timestamp': self.timestamp.isoformat()
        }

class ConnectionError(EagleEyeError):
    """连接错误"""
    
    def __init__(self, message: str, ip_address: str = None):
        super().__init__(
            message,
            error_code="CONNECTION_ERROR",
            details={'ip_address': ip_address}
        )

class DataProcessingError(EagleEyeError):
    """数据处理错误"""
    
    def __init__(self, message: str, file_path: str = None, process_type: str = None):
        super().__init__(
            message,
            error_code="DATA_PROCESSING_ERROR",
            details={
                'file_path': file_path,
                'process_type': process_type
            }
        )

class APIError(EagleEyeError):
    """API错误"""
    
    def __init__(self, message: str, api_name: str = None, status_code: int = None):
        super().__init__(
            message,
            error_code="API_ERROR",
            details={
                'api_name': api_name,
                'status_code': status_code
            }
        )

class ConfigurationError(EagleEyeError):
    """配置错误"""
    
    def __init__(self, message: str, config_key: str = None):
        super().__init__(
            message,
            error_code="CONFIGURATION_ERROR",
            details={'config_key': config_key}
        )

class StorageError(EagleEyeError):
    """存储错误"""
    
    def __init__(self, message: str, file_path: str = None, operation: str = None):
        super().__init__(
            message,
            error_code="STORAGE_ERROR",
            details={
                'file_path': file_path,
                'operation': operation
            }
        )

def handle_errors(func):
    """错误处理装饰器"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except EagleEyeError as e:
            logger.error(f"业务错误: {e.message}", extra={'error_details': e.details})
            raise
        except Exception as e:
            logger.error(f"系统错误: {str(e)}", extra={'traceback': traceback.format_exc()})
            raise EagleEyeError(
                message=f"系统错误: {str(e)}",
                error_code="SYSTEM_ERROR",
                details={'original_error': str(e)}
            )
    return wrapper

def log_api_call(endpoint: str, method: str = "GET"):
    """API调用日志装饰器"""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            start_time = datetime.now()
            logger.info(f"API调用开始: {method} {endpoint}")
            
            try:
                result = func(*args, **kwargs)
                duration = (datetime.now() - start_time).total_seconds()
                logger.info(f"API调用成功: {method} {endpoint} - 耗时: {duration:.3f}s")
                return result
            except Exception as e:
                duration = (datetime.now() - start_time).total_seconds()
                logger.error(f"API调用失败: {method} {endpoint} - 耗时: {duration:.3f}s - 错误: {str(e)}")
                raise
        return wrapper
    return decorator

def log_performance(operation: str):
    """性能监控装饰器"""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            start_time = datetime.now()
            memory_before = get_memory_usage()
            
            try:
                result = func(*args, **kwargs)
                duration = (datetime.now() - start_time).total_seconds()
                memory_after = get_memory_usage()
                
                logger.info(f"性能监控: {operation} - 耗时: {duration:.3f}s - 内存: {memory_after - memory_before:.2f}MB")
                return result
            except Exception as e:
                duration = (datetime.now() - start_time).total_seconds()
                logger.error(f"性能监控: {operation} - 失败 - 耗时: {duration:.3f}s - 错误: {str(e)}")
                raise
        return wrapper
    return decorator

def get_memory_usage() -> float:
    """获取内存使用量（MB）"""
    try:
        import psutil
        process = psutil.Process()
        return process.memory_info().rss / 1024 / 1024
    except ImportError:
        return 0.0

def setup_logging(level: str = "INFO"):
    """设置日志级别"""
    numeric_level = getattr(logging, level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f'Invalid log level: {level}')
    
    logger.setLevel(numeric_level)
    
    # 更新所有处理器的级别
    for handler in logger.handlers:
        handler.setLevel(numeric_level)
    
    logger.info(f"日志级别设置为: {level}")

def get_error_response(error: EagleEyeError) -> Dict[str, Any]:
    """获取标准错误响应"""
    return {
        'success': False,
        'error': error.to_dict(),
        'data': None
    }

def get_success_response(data: Any = None, message: str = "操作成功") -> Dict[str, Any]:
    """获取标准成功响应"""
    return {
        'success': True,
        'message': message,
        'data': data,
        'timestamp': datetime.now().isoformat()
    }

class ErrorCollector:
    """错误收集器"""
    
    def __init__(self, max_errors: int = 1000):
        self.max_errors = max_errors
        self.errors = []
    
    def add_error(self, error: EagleEyeError):
        """添加错误"""
        self.errors.append(error)
        
        # 保持错误数量在限制内
        if len(self.errors) > self.max_errors:
            self.errors = self.errors[-self.max_errors:]
    
    def get_recent_errors(self, limit: int = 10) -> list:
        """获取最近的错误"""
        return [error.to_dict() for error in self.errors[-limit:]]
    
    def get_error_statistics(self) -> Dict[str, Any]:
        """获取错误统计"""
        if not self.errors:
            return {}
        
        error_counts = {}
        for error in self.errors:
            error_counts[error.error_code] = error_counts.get(error.error_code, 0) + 1
        
        return {
            'total_errors': len(self.errors),
            'error_counts': error_counts,
            'most_common_error': max(error_counts.items(), key=lambda x: x[1])[0]
        }
    
    def clear_errors(self):
        """清除所有错误"""
        self.errors.clear()

# 全局错误收集器
error_collector = ErrorCollector()

# 监控系统健康状态
def get_system_health() -> Dict[str, Any]:
    """获取系统健康状态"""
    try:
        health = {
            'status': 'healthy',
            'timestamp': datetime.now().isoformat(),
            'memory_usage': get_memory_usage(),
            'disk_usage': get_disk_usage(),
            'recent_errors': error_collector.get_recent_errors(5)
        }
        
        # 检查内存使用
        if health['memory_usage'] > 1024:  # 1GB
            health['status'] = 'warning'
            health['warnings'] = ['内存使用过高']
        
        # 检查磁盘使用
        if health['disk_usage'] > 90:  # 90%
            health['status'] = 'warning'
            health['warnings'] = health.get('warnings', []) + ['磁盘使用过高']
        
        return health
        
    except Exception as e:
        logger.error(f"获取系统健康状态失败: {str(e)}")
        return {
            'status': 'error',
            'error': str(e),
            'timestamp': datetime.now().isoformat()
        }

def get_disk_usage() -> float:
    """获取磁盘使用率（百分比）"""
    try:
        import shutil
        total, used, free = shutil.disk_usage(".")
        return (used / total) * 100
    except Exception:
        return 0.0
