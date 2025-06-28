import cantools
import can
import sys

def main():
    if len(sys.argv) != 2:
        print(f"用法: {sys.argv[0]} <dbc_file>")
        sys.exit(1)

    dbc_file = sys.argv[1]
    
    try:
        # 加载DBC文件
        db = cantools.database.load_file(dbc_file)
        print(f"成功加载DBC文件: {dbc_file}")
    except Exception as e:
        print(f"加载DBC文件失败: {e}")
        sys.exit(1)

    try:
        # 创建CAN总线接口，连接到can0
        bus = can.interface.Bus(
            bustype='socketcan',
            channel='can0',
            bitrate=500000  # 比特率，根据实际情况调整
        )
        print("成功连接到CAN0接口")
        print("开始接收和解析CAN消息...")
        print("-" * 60)
    except Exception as e:
        print(f"连接CAN0接口失败: {e}")
        sys.exit(1)

    try:
        while True:
            # 接收CAN消息
            message = bus.recv(1.0)  # 超时时间1秒
            
            if message is not None:
                try:
                    # 解析CAN消息
                    can_id = message.arbitration_id
                    data = message.data
                    
                    # 根据CAN ID查找消息定义
                    msg_def = db.get_message_by_frame_id(can_id)
                    
                    # 解析信号值
                    signals = msg_def.decode(data)
                    
                    # 打印解析结果
                    print(f"ID: 0x{can_id:03X} ({msg_def.name})")
                    for signal_name, value in signals.items():
                        print(f"  {signal_name}: {value}")
                    print("-" * 60)
                    
                except KeyError:
                    # 未在DBC中定义的CAN ID
                    print(f"未定义的CAN ID: 0x{can_id:03X}")
                    print("-" * 60)
                except Exception as e:
                    print(f"解析错误: {e}")
                    print("-" * 60)
                    
    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        # 关闭CAN总线连接
        bus.shutdown()
        print("已断开CAN0连接")

if __name__ == "__main__":
    main()