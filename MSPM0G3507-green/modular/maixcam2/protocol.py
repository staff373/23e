import config  # 导入协议字段使用的图像尺寸配置。

g_rx_buffer = ""  # 保存跨轮次残留的串口接收文本。

def send_line(serial, line):  # 发送一行协议文本。
    serial.write_str(line + "\r\n")  # 按协议约定补齐 CRLF 行尾。

def send_ack(serial, seq, cmd, ok, err_code):  # 发送 ACK,cmd,ok,err 回包。
    send_line(serial, "<C,{},{},{},{},{}".format(seq, "ACK", cmd, ok, err_code))  # 输出命令确认包。

def send_err(serial, seq, err_code):  # 发送 ERR,code 回包。
    send_line(serial, "<C,{},{},{}".format(seq, "ERR", err_code))  # 输出视觉侧错误码。

def send_status(serial, seq, mode, fps10, frame_id, err_code):  # 发送 STATUS,mode,fps10,frame,err 回包。
    send_line(serial, "<C,{},{},{},{},{},{}".format(seq, "STATUS", mode, fps10, frame_id, err_code))  # 输出低频状态包。

def send_track(serial, seq, frame_id, track_valid, red_valid, green_valid, dist_valid, red_x, red_y, green_x, green_y, err_x, err_y, dist_mm, red_conf, green_conf, latency_ms, err_code):  # 发送 TRACK 双光斑观测包。
    send_line(serial, "<C,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format(seq, "TRACK", frame_id, config.TRACK_IMG_W, config.TRACK_IMG_H, track_valid, red_valid, green_valid, dist_valid, red_x, red_y, green_x, green_y, err_x, err_y, dist_mm, red_conf, green_conf, latency_ms, err_code))  # 输出红绿光斑坐标、像素误差、距离和耗时。

def parse_line(line):  # 解析一行 TI 命令文本。
    parts = line.strip().split(",")  # 按逗号拆分 ASCII CSV 字段。
    if len(parts) < 3 or parts[0] != ">T":  # 检查方向头和最小字段数。
        return ("", "BAD_PACKET", [])  # 返回坏包事件交给 main.py 决定错误处理。
    return (parts[1], parts[2], parts[3:])  # 返回 seq、cmd 和参数列表。

def decode_uart_data(data):  # 把串口 read 返回值转换为文本。
    if isinstance(data, str):  # 兼容固件返回字符串的情况。
        return data  # 直接返回字符串数据。
    return data.decode("utf-8", "ignore")  # 兼容固件返回 bytes 的情况并忽略坏字节。

def poll_commands(serial):  # 非阻塞轮询串口并返回完整命令事件列表。
    global g_rx_buffer  # 声明要修改接收缓存。
    events = []  # 创建本轮解析出的命令事件列表。
    data = serial.read()  # 非阻塞读取当前可用串口数据。
    if not data:  # 判断本轮是否没有收到数据。
        return events  # 没有数据时返回空事件列表。
    g_rx_buffer = g_rx_buffer + decode_uart_data(data)  # 追加新收到的文本到缓存。
    while "\n" in g_rx_buffer:  # 循环处理所有完整行。
        line, g_rx_buffer = g_rx_buffer.split("\n", 1)  # 切出一行并保留剩余缓存。
        line = line.strip("\r")  # 去掉 CR 字符。
        if line:  # 忽略空行。
            events.append(parse_line(line))  # 解析并保存该行命令事件。
    return events  # 返回本轮所有命令事件。
