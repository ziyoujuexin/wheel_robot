import serial
import time

# 配置串口
SERIAL_PORT = '/dev/ttyCH341USB1'  # 串口设备路径，根据实际情况修改
BAUD_RATE = 115200            # 波特率，根据硬件配置选择
TIMEOUT = 1                   # 超时时间

# 连接硬件
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    
    # 检查串口是否打开
    if ser.is_open:
        print(f"串口 {SERIAL_PORT} 已成功打开！")
    else:
        print(f"无法打开串口 {SERIAL_PORT}。")
        exit(1)
except serial.SerialException as e:
    print(f"打开串口时出错: {e}")
    exit(1)

# UartProtocol 配置
FRAME_HEADER = 0x55AA
FRAME_TAIL = 0xBB
SENDER_ID = 0x01
DATA_TYPE_MOTION = 0x02
FRAME_LENGTH = 7

def send_frame(data_type, payload):
    """发送数据帧"""
    payload_len = len(payload)
    frame_len = FRAME_LENGTH + payload_len

    # 构建帧头
    frame = bytearray()
    frame.append((FRAME_HEADER >> 8) & 0xFF)
    frame.append(FRAME_HEADER & 0xFF)

    # 构建ID、数据类型、长度
    frame.append(SENDER_ID)
    frame.append(data_type)
    frame.append(payload_len)

    # 添加数据负载
    frame.extend(payload)

    # 计算校验和
    checksum = 0
    for byte in frame:
        checksum += byte
    frame.append(checksum & 0xFF)

    # 添加帧尾
    frame.append(FRAME_TAIL)

    # 发送数据
    ser.write(frame)
    print(f"发送帧: {frame.hex()}")

def play_animation(anim_name):
    """播放动画"""
    anim_name_cstr = anim_name.encode('utf-8')  # 转换为字节流
    payload = bytearray([0xD0])  # 动画播放标识
    payload.extend(anim_name_cstr)

    send_frame(DATA_TYPE_MOTION, payload)

if __name__ == "__main__":
    try:
        animation_name = "[L]开心_8_36s_x1.1"  # 动画名称，根据需要修改
        print(f"播放动画: {animation_name}")
        play_animation(animation_name)
        time.sleep(10)  # 等待动画播放
        animation_name = "[L]生气_7_41s_x1.2"
        play_animation(animation_name)
        time.sleep(5)  # 等待动画播放

        
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        ser.close()
        print("串口已关闭")
