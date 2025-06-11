"""
MuJoCo MCP服务器管理模块
"""
import logging
import threading
from typing import Optional

from .server import MuJoCoMCPServer
from .enhanced_auth_manager import EnhancedAuthManager

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger("mujoco_mcp")

# 全局服务器实例
_server_instance: Optional[MuJoCoMCPServer] = None
_server_thread: Optional[threading.Thread] = None

def start(host: str = "localhost", 
          port: int = 7777, 
          blocking: bool = True, 
          auth_manager: Optional[EnhancedAuthManager] = None,
          auto_approve: bool = True) -> MuJoCoMCPServer:
    """
    启动MuJoCo MCP服务器
    
    Args:
        host: 服务器主机名
        port: 服务器端口
        blocking: 是否阻塞当前线程
        auth_manager: 自定义授权管理器，如果为None则使用默认管理器
        auto_approve: 是否自动批准所有请求 (仅当使用默认授权管理器时有效)
        
    Returns:
        MuJoCoMCPServer: 服务器实例
    """
    global _server_instance, _server_thread
    
    if _server_instance is not None:
        logger.warning("服务器已在运行，正在停止现有实例")
        stop()
    
    # 如果未提供授权管理器，使用默认管理器并设置自动批准模式
    if auth_manager is None:
        auth_manager = EnhancedAuthManager(auto_approve_mode=auto_approve)
    
    logger.info(f"正在启动MuJoCo MCP服务器 {host}:{port}" + 
               f" (自动批准: {'启用' if auth_manager.auto_approve_mode else '禁用'})")
    
    _server_instance = MuJoCoMCPServer(host=host, port=port, auth_manager=auth_manager)
    
    if blocking:
        # 阻塞模式：在当前线程运行服务器
        _server_instance.start()
    else:
        # 非阻塞模式：在新线程运行服务器
        _server_thread = threading.Thread(
            target=_server_instance.start,
            name="MuJoCoMCPServerThread",
            daemon=True
        )
        _server_thread.start()
    
    return _server_instance

def stop() -> None:
    """停止MuJoCo MCP服务器（如果正在运行）"""
    global _server_instance, _server_thread
    
    if _server_instance is not None:
        logger.info("正在停止MuJoCo MCP服务器")
        _server_instance.shutdown()
        
        if _server_thread is not None and _server_thread.is_alive():
            _server_thread.join(timeout=2.0)
            if _server_thread.is_alive():
                logger.warning("服务器线程未能在超时内退出")
        
        _server_instance = None
        _server_thread = None
    else:
        logger.warning("没有正在运行的服务器实例")

def get_server() -> Optional[MuJoCoMCPServer]:
    """
    获取当前服务器实例
    
    Returns:
        Optional[MuJoCoMCPServer]: 服务器实例，未运行时返回None
    """
    return _server_instance

def is_running() -> bool:
    """
    检查服务器是否正在运行
    
    Returns:
        bool: 服务器是否正在运行
    """
    return _server_instance is not None

def set_auto_approve(enabled: bool) -> None:
    """
    设置自动批准模式
    
    Args:
        enabled: 是否启用自动批准
    """
    if _server_instance is not None and _server_instance.auth_manager is not None:
        _server_instance.auth_manager.set_auto_approve_mode(enabled)
        logger.info(f"已{'启用' if enabled else '禁用'}自动批准模式")
    else:
        logger.warning("服务器未运行或未配置授权管理器")

def add_trusted_operation(operation: str) -> None:
    """
    添加信任的操作（无需批准）
    
    Args:
        operation: 操作名称
    """
    if _server_instance is not None and _server_instance.auth_manager is not None:
        _server_instance.auth_manager.add_trusted_operation(operation)
    else:
        logger.warning("服务器未运行或未配置授权管理器")

def add_approved_client(client_id: str) -> None:
    """
    添加批准的客户端
    
    Args:
        client_id: 客户端ID
    """
    if _server_instance is not None and _server_instance.auth_manager is not None:
        _server_instance.auth_manager.add_approved_client(client_id)
    else:
        logger.warning("服务器未运行或未配置授权管理器")

def get_pending_requests():
    """
    获取所有待处理的授权请求
    
    Returns:
        List[Dict]: 待处理请求列表
    """
    if _server_instance is not None and _server_instance.auth_manager is not None:
        return _server_instance.auth_manager.get_pending_requests()
    else:
        logger.warning("服务器未运行或未配置授权管理器")
        return []

def approve_request(request_id: str, message: Optional[str] = None) -> bool:
    """
    批准授权请求
    
    Args:
        request_id: 请求ID
        message: 响应消息 (可选)
        
    Returns:
        bool: 是否成功批准
    """
    if _server_instance is not None and _server_instance.auth_manager is not None:
        return _server_instance.auth_manager.approve_request(request_id, message)
    else:
        logger.warning("服务器未运行或未配置授权管理器")
        return False

def reject_request(request_id: str, message: Optional[str] = None) -> bool:
    """
    拒绝授权请求
    
    Args:
        request_id: 请求ID
        message: 响应消息 (可选)
        
    Returns:
        bool: 是否成功拒绝
    """
    if _server_instance is not None and _server_instance.auth_manager is not None:
        return _server_instance.auth_manager.reject_request(request_id, message)
    else:
        logger.warning("服务器未运行或未配置授权管理器")
        return False

def get_active_simulations():
    """
    获取所有活动模拟的ID
    
    Returns:
        List[str]: 模拟ID列表
    """
    if _server_instance is not None:
        return _server_instance.get_simulation_ids()
    else:
        logger.warning("服务器未运行")
        return [] 