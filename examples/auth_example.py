"""
授权管理示例 - 展示如何使用MuJoCo MCP的授权功能
"""
import sys
import os
import time
import uuid
import logging
import threading
import json
from typing import Dict, List, Optional

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.mujoco_mcp.server_manager import start, stop, get_pending_requests, approve_request, reject_request, set_auto_approve
from src.mujoco_mcp.auth_manager import AuthManager
from model_context_protocol.client import MCPClient

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger("auth_example")

class AuthorizationMonitor:
    """授权监控类 - 监控和处理授权请求"""
    
    def __init__(self, check_interval: float = 1.0):
        """
        初始化授权监控器
        
        Args:
            check_interval: 检查请求的间隔时间(秒)
        """
        self.check_interval = check_interval
        self.running = False
        self.monitor_thread = None
        self.pending_requests = {}  # 保存未处理的请求
        self.auto_approve_list = set()  # 自动批准的操作列表
        self.auto_reject_list = set()  # 自动拒绝的操作列表
    
    def start_monitoring(self):
        """启动授权监控"""
        if self.running:
            logger.warning("监控器已在运行")
            return
        
        self.running = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="AuthMonitorThread",
            daemon=True
        )
        self.monitor_thread.start()
        logger.info("已启动授权监控")
    
    def stop_monitoring(self):
        """停止授权监控"""
        if not self.running:
            return
        
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        
        logger.info("已停止授权监控")
    
    def add_auto_approve_operation(self, operation: str):
        """添加自动批准的操作"""
        self.auto_approve_list.add(operation)
        logger.info(f"已添加自动批准操作: {operation}")
    
    def add_auto_reject_operation(self, operation: str):
        """添加自动拒绝的操作"""
        self.auto_reject_list.add(operation)
        logger.info(f"已添加自动拒绝操作: {operation}")
    
    def _monitor_loop(self):
        """监控线程主循环"""
        while self.running:
            try:
                # 获取待处理的请求
                requests = get_pending_requests()
                
                for req in requests:
                    req_id = req["request_id"]
                    
                    # 跳过已经处理过的请求
                    if req_id in self.pending_requests:
                        continue
                    
                    # 保存新请求
                    self.pending_requests[req_id] = req
                    
                    # 处理请求
                    self._process_request(req)
                
                # 清理已处理的请求
                self._cleanup_processed_requests(requests)
                
            except Exception as e:
                logger.error(f"监控请求时出错: {str(e)}")
            
            # 等待下一次检查
            time.sleep(self.check_interval)
    
    def _process_request(self, request: Dict):
        """
        处理授权请求
        
        Args:
            request: 授权请求字典
        """
        req_id = request["request_id"]
        client_id = request["client_id"]
        operation = request["operation"]
        
        # 打印新请求信息
        logger.info(f"收到新的授权请求: ID={req_id}, 客户端={client_id}, 操作={operation}")
        
        # 根据规则自动处理
        if operation in self.auto_approve_list:
            approve_request(req_id, "自动批准的操作")
            logger.info(f"已自动批准请求: {req_id}")
        elif operation in self.auto_reject_list:
            reject_request(req_id, "自动拒绝的操作")
            logger.info(f"已自动拒绝请求: {req_id}")
        else:
            # 等待手动处理
            logger.info(f"请求 {req_id} 等待手动处理")
    
    def _cleanup_processed_requests(self, current_requests: List[Dict]):
        """
        清理已处理的请求
        
        Args:
            current_requests: 当前待处理的请求列表
        """
        current_ids = {req["request_id"] for req in current_requests}
        processed_ids = [req_id for req_id in self.pending_requests if req_id not in current_ids]
        
        for req_id in processed_ids:
            del self.pending_requests[req_id]
    
    def print_pending_requests(self):
        """打印当前所有待处理的请求"""
        if not self.pending_requests:
            print("没有待处理的请求")
            return
        
        print("\n当前待处理的请求:")
        print("=" * 60)
        for req_id, req in self.pending_requests.items():
            print(f"请求ID: {req_id}")
            print(f"客户端: {req['client_id']}")
            print(f"操作: {req['operation']}")
            if req.get("resource"):
                print(f"资源: {req['resource']}")
            if req.get("parameters"):
                print(f"参数: {json.dumps(req['parameters'], indent=2, ensure_ascii=False)}")
            print(f"请求时间: {time.ctime(req['requested_at'])}")
            print("-" * 60)

    def manual_approve(self, request_id: str):
        """手动批准请求"""
        if request_id not in self.pending_requests:
            print(f"找不到待处理的请求: {request_id}")
            return False
        
        if approve_request(request_id, "手动批准"):
            print(f"已批准请求: {request_id}")
            del self.pending_requests[request_id]
            return True
        else:
            print(f"批准请求失败: {request_id}")
            return False
    
    def manual_reject(self, request_id: str):
        """手动拒绝请求"""
        if request_id not in self.pending_requests:
            print(f"找不到待处理的请求: {request_id}")
            return False
        
        if reject_request(request_id, "手动拒绝"):
            print(f"已拒绝请求: {request_id}")
            del self.pending_requests[request_id]
            return True
        else:
            print(f"拒绝请求失败: {request_id}")
            return False

def run_client(host: str, port: int, client_id: str, auto_wait: bool = False):
    """
    运行MCP客户端并发送请求
    
    Args:
        host: 服务器主机名
        port: 服务器端口
        client_id: 客户端ID
        auto_wait: 是否自动等待请求处理
    """
    logger.info(f"启动客户端 {client_id}")
    
    try:
        # 连接MCP服务器
        client = MCPClient(host, port)
        client.connect()
        logger.info(f"客户端 {client_id} 已连接")
        
        # 创建简单机器人模型XML
        model_xml = """
        <mujoco>
            <worldbody>
                <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
                <geom type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>
                <body pos="0 0 1" name="robot">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
                </body>
            </worldbody>
        </mujoco>
        """
        
        # 启动新的模拟
        logger.info(f"客户端 {client_id} 请求启动模拟")
        result = client.execute_tool("start_simulation", {"model_xml": model_xml, "client_id": client_id})
        
        if "error" in result:
            logger.error(f"启动模拟失败: {result['error']}")
            return
        
        sim_id = result["sim_id"]
        logger.info(f"模拟已启动: {sim_id}")
        
        # 如果需要等待授权请求处理
        if auto_wait:
            logger.info("等待10秒，让授权请求被处理...")
            time.sleep(10)
        
        # 获取模拟信息
        logger.info(f"客户端 {client_id} 请求模拟信息")
        info = client.get_resource("simulation_info", {"sim_id": sim_id, "client_id": client_id})
        
        if "error" in info:
            logger.error(f"获取模拟信息失败: {info['error']}")
        else:
            logger.info(f"模拟信息: {json.dumps(info, indent=2)}")
        
        # 推进模拟
        logger.info(f"客户端 {client_id} 请求步进模拟")
        step_result = client.execute_tool("step_simulation", {"sim_id": sim_id, "num_steps": 10, "client_id": client_id})
        
        if "error" in step_result:
            logger.error(f"步进模拟失败: {step_result['error']}")
        else:
            logger.info(f"已步进模拟: {step_result}")
        
        # 删除模拟
        logger.info(f"客户端 {client_id} 请求删除模拟")
        delete_result = client.execute_tool("delete_simulation", {"sim_id": sim_id, "client_id": client_id})
        
        if "error" in delete_result:
            logger.error(f"删除模拟失败: {delete_result['error']}")
        else:
            logger.info(f"已删除模拟: {delete_result}")
            
    except Exception as e:
        logger.error(f"客户端 {client_id} 出错: {str(e)}")
    finally:
        # 断开连接
        client.disconnect()
        logger.info(f"客户端 {client_id} 已断开连接")

def main():
    """主函数"""
    print("=== MuJoCo MCP 授权管理示例 ===")
    
    # 创建自定义授权管理器
    auth_manager = AuthManager(auto_approve_mode=False)  # 禁用自动批准模式
    
    # 添加一些预批准的操作
    auth_manager.add_trusted_operation("get_resource")  # 信任所有资源请求
    
    # 创建并启动授权监控器
    monitor = AuthorizationMonitor()
    monitor.add_auto_approve_operation("step_simulation")  # 自动批准步进操作
    monitor.add_auto_reject_operation("delete_simulation")  # 自动拒绝删除操作
    monitor.start_monitoring()
    
    # 启动服务器
    host = "localhost"
    port = 7777
    print(f"启动MuJoCo MCP服务器在 {host}:{port}")
    server = start(host=host, port=port, blocking=False, auth_manager=auth_manager)
    
    try:
        # 模拟交互式授权管理系统
        print("\n正在启动授权管理控制台，您可以在这里管理授权请求")
        print("在此期间，我们将启动一个测试客户端来发送请求")
        
        # 创建一个测试客户端线程
        client_id = f"test_client_{uuid.uuid4().hex[:8]}"
        client_thread = threading.Thread(
            target=run_client,
            args=(host, port, client_id, False),
            daemon=True
        )
        client_thread.start()
        
        # 交互式控制台
        while True:
            print("\n可用命令:")
            print("1 - 显示待处理的请求")
            print("2 - 批准请求")
            print("3 - 拒绝请求")
            print("4 - 启动另一个测试客户端")
            print("5 - 切换自动批准模式")
            print("q - 退出")
            
            cmd = input("请输入命令: ").strip()
            
            if cmd == "q":
                break
            elif cmd == "1":
                monitor.print_pending_requests()
            elif cmd == "2":
                req_id = input("请输入要批准的请求ID: ").strip()
                monitor.manual_approve(req_id)
            elif cmd == "3":
                req_id = input("请输入要拒绝的请求ID: ").strip()
                monitor.manual_reject(req_id)
            elif cmd == "4":
                client_id = f"test_client_{uuid.uuid4().hex[:8]}"
                client_thread = threading.Thread(
                    target=run_client,
                    args=(host, port, client_id, False),
                    daemon=True
                )
                client_thread.start()
                print(f"已启动新的测试客户端: {client_id}")
            elif cmd == "5":
                current_mode = auth_manager.auto_approve_mode
                set_auto_approve(not current_mode)
                print(f"已{'启用' if not current_mode else '禁用'}自动批准模式")
            else:
                print("无效的命令")
    
    except KeyboardInterrupt:
        print("\n接收到中断信号，正在退出...")
    finally:
        # 停止监控和服务器
        monitor.stop_monitoring()
        stop()
        print("已退出授权管理示例")

if __name__ == "__main__":
    main()