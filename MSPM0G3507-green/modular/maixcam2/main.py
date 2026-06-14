from maix import app, camera, display, err, image, pinmap, time, uart  # 导入 MaixPy 运行、相机、显示、错误、图像、引脚、时间和串口模块。
import config  # 导入现场可调参数配置模块。
import math  # 导入平方根计算模块，用于把红绿像素误差换算成临时等效毫米距离。
import protocol  # 导入项目通信协议封包、解包和串口轮询模块。
import vision_green  # 导入绿色激光点检测模块。
import vision_spot  # 导入红色激光点检测模块，保持与红色工程同源。

VISION_IDLE = "VISION_IDLE"  # 定义视觉空闲状态。
VISION_TRACK640 = "VISION_TRACK640"  # 定义 640x480 红绿双光斑检测状态。
VISION_ERROR = "VISION_ERROR"  # 定义视觉错误状态。

ERR_NONE = 0  # 定义无错误码。
ERR_BAD_PACKET = 1  # 定义协议包格式错误码。
ERR_BAD_CMD = 2  # 定义未知命令错误码。
ERR_BAD_MODE = 3  # 定义未知模式错误码。
ERR_PROCESS = 4  # 定义图像采集或处理错误码。

g_serial = None  # 保存串口对象。
g_camera = None  # 保存相机对象。
g_display = None  # 保存显示对象。
g_state = VISION_TRACK640  # 保存当前视觉状态，当前上电默认直接进入 TRACK640 便于串口 smoke 和追踪联调。
g_seq = "0"  # 保存最近一次 TI 命令序号。
g_frame_id = 0  # 保存真实处理过的相机帧号。
g_fps10 = 0  # 保存视觉处理 FPS 乘以 10 的整数值。
g_last_frame_ms = 0  # 保存上一帧相机处理开始时间。
g_last_latency_ms = 0  # 保存上一帧采集加识别耗时。
g_last_track_ms = 0  # 保存上一帧 TRACK 发送时间。
g_last_status_ms = 0  # 保存上一帧 STATUS 发送时间。
g_last_sent_frame_id = 0  # 保存上一帧已发送的相机帧号。
g_track_accum_ms = 0  # 保存 TRACK 目标 50Hz 发送调度累计时间。
g_track_ready = False  # 保存当前相机帧是否应该发送 TRACK。
g_track_valid = 0  # 保存红绿双光斑同时有效标志。
g_red_valid = 0  # 保存红点检测有效标志。
g_green_valid = 0  # 保存绿点检测有效标志。
g_dist_valid = 0  # 保存距离毫米值是否有效。
g_red_x = -1  # 保存红点 X 像素坐标。
g_red_y = -1  # 保存红点 Y 像素坐标。
g_green_x = -1  # 保存绿点 X 像素坐标。
g_green_y = -1  # 保存绿点 Y 像素坐标。
g_err_x = 0  # 保存像素误差 X，定义为 red_x - green_x。
g_err_y = 0  # 保存像素误差 Y，定义为 red_y - green_y。
g_pixel_dist2 = 0  # 保存红绿像素距离平方，用于 10px 等效 3cm 的临时判定。
g_track_locked = 0  # 保存临时像素阈值锁定标志，像素距离小于等于 TRACK_3CM_PIXEL 时为 1。
g_dist_mm = -1  # 保存按 10px 等效 3cm 临时换算得到的红绿距离毫米值。
g_red_conf = 0  # 保存红点置信度。
g_green_conf = 0  # 保存绿点置信度。
g_track_err = ERR_NONE  # 保存当前 TRACK 错误码。
g_last_red_result = None  # 保存上一帧红点检测结果供 overlay 使用。
g_last_green_result = None  # 保存上一帧绿点检测结果供 overlay 使用。

def now_ms():  # 获取当前毫秒时间。
    return time.ticks_ms()  # 返回 MaixPy 毫秒计数。

def elapsed_ms(start_ms, current_ms):  # 计算两个毫秒时间之间的差值。
    return current_ms - start_ms  # 返回当前时间减起始时间。

def setup_uart():  # 初始化 MaixCAM2 UART2。
    err.check_raise(pinmap.set_pin_function(config.UART_TX_PIN, config.UART_TX_FUNC), "set UART TX failed")  # 按配置复用 TX 引脚。
    err.check_raise(pinmap.set_pin_function(config.UART_RX_PIN, config.UART_RX_FUNC), "set UART RX failed")  # 按配置复用 RX 引脚。
    return uart.UART(config.UART_DEVICE, config.UART_BAUD)  # 按配置打开 UART 串口对象。

def setup_camera():  # 初始化固定 640x480 相机和显示。
    global g_camera, g_display  # 声明要修改相机和显示对象。
    if not g_camera:  # 判断相机是否尚未初始化。
        g_camera = camera.Camera(config.TRACK_IMG_W, config.TRACK_IMG_H, fps=config.CAMERA_FPS)  # 固定使用 640x480 相机，避免运行中重建相机。
    if not g_display:  # 判断显示是否尚未初始化。
        g_display = display.Display()  # 创建 MaixCAM2 显示对象。

def read_camera_frame():  # 安全读取一帧相机图像。
    try:  # 捕获板端 camera.read 超时或底层返回空指针异常。
        return g_camera.read()  # 读取一帧固定 640x480 图像。
    except Exception:  # 处理相机读取失败，避免主程序退出。
        set_track_error(ERR_PROCESS)  # 记录处理错误并进入可恢复错误状态。
        return None  # 返回空图像让当前帧处理提前结束。

def current_error():  # 获取当前状态错误码。
    if g_state == VISION_ERROR:  # 判断当前是否处于错误状态。
        return g_track_err  # 返回当前 TRACK 错误码。
    return ERR_NONE  # 非错误状态返回无错误。

def send_ack(seq, cmd, ok, err_code):  # 发送命令确认包。
    protocol.send_ack(g_serial, seq, cmd, ok, err_code)  # 通过 protocol.py 输出 ACK,cmd,ok,err。

def send_err(seq, err_code):  # 发送视觉错误包。
    protocol.send_err(g_serial, seq, err_code)  # 通过 protocol.py 输出 ERR,code。

def send_status(seq):  # 发送当前视觉状态包。
    protocol.send_status(g_serial, seq, g_state, g_fps10, g_frame_id, current_error())  # 通过 protocol.py 输出 STATUS,mode,fps10,frame,err。

def send_track(seq):  # 发送双光斑坐标包。
    protocol.send_track(g_serial, seq, g_frame_id, g_track_valid, g_red_valid, g_green_valid, g_dist_valid, g_red_x, g_red_y, g_green_x, g_green_y, g_err_x, g_err_y, g_dist_mm, g_red_conf, g_green_conf, g_last_latency_ms, g_track_err)  # 通过 protocol.py 输出 TRACK 字段。

def set_track_result(red_result, green_result, latency_ms):  # 汇总红绿检测结果给协议输出。
    global g_track_valid, g_red_valid, g_green_valid, g_dist_valid, g_red_x, g_red_y, g_green_x, g_green_y, g_err_x, g_err_y, g_pixel_dist2, g_track_locked, g_dist_mm, g_red_conf, g_green_conf, g_track_err, g_last_latency_ms, g_last_red_result, g_last_green_result  # 声明要修改 TRACK 全局结果。
    g_last_red_result = red_result  # 保存红点结果供 overlay 使用。
    g_last_green_result = green_result  # 保存绿点结果供 overlay 使用。
    g_red_valid = red_result["valid"]  # 更新红点有效标志。
    g_green_valid = green_result["valid"]  # 更新绿点有效标志。
    g_red_x = red_result["x"]  # 更新红点 X 坐标。
    g_red_y = red_result["y"]  # 更新红点 Y 坐标。
    g_green_x = green_result["x"]  # 更新绿点 X 坐标。
    g_green_y = green_result["y"]  # 更新绿点 Y 坐标。
    g_red_conf = red_result["conf"]  # 更新红点置信度。
    g_green_conf = green_result["conf"]  # 更新绿点置信度。
    g_track_valid = 1 if g_red_valid and g_green_valid else 0  # 只有红绿双点都有效时标记 TRACK 有效。
    if g_track_valid:  # 判断红绿双点是否同时有效。
        g_err_x = g_red_x - g_green_x  # 计算 X 像素误差，定义为 red - green。
        g_err_y = g_red_y - g_green_y  # 计算 Y 像素误差，定义为 red - green。
        g_pixel_dist2 = g_err_x * g_err_x + g_err_y * g_err_y  # 计算红绿中心像素距离平方。
        pixel_dist = int(math.sqrt(g_pixel_dist2) + 0.5)  # 计算四舍五入后的红绿中心像素距离。
        g_dist_valid = 1  # 红绿双点有效时启用临时等效毫米距离字段。
        g_dist_mm = int(pixel_dist * config.TRACK_3CM_MM / config.TRACK_3CM_PIXEL + 0.5)  # 按 10px 等效 30mm 临时换算 dist_mm。
        g_track_locked = 1 if g_pixel_dist2 <= config.TRACK_3CM_PIXEL * config.TRACK_3CM_PIXEL else 0  # 使用 10px 临时阈值判断是否等效达标。
    else:  # 处理任一光斑无效的情况。
        g_err_x = 0  # 清空 X 像素误差，避免 TI 使用旧误差。
        g_err_y = 0  # 清空 Y 像素误差，避免 TI 使用旧误差。
        g_pixel_dist2 = 0  # 清空像素距离平方，避免 overlay 显示旧距离。
        g_track_locked = 0  # 清空临时锁定标志。
        g_dist_valid = 0  # 红绿双点不同时有效时距离字段无效。
        g_dist_mm = -1  # 红绿双点不同时有效时使用 -1 表示无临时距离。
    g_last_latency_ms = latency_ms  # 更新采集加识别耗时。
    g_track_err = red_result["err"] if red_result["err"] != ERR_NONE else green_result["err"]  # 优先记录非零视觉错误码。

def clear_track_result(err_code):  # 清空双光斑输出结果。
    global g_track_valid, g_red_valid, g_green_valid, g_dist_valid, g_red_x, g_red_y, g_green_x, g_green_y, g_err_x, g_err_y, g_pixel_dist2, g_track_locked, g_dist_mm, g_red_conf, g_green_conf, g_track_err  # 声明要修改 TRACK 全局结果。
    g_track_valid = 0  # 清空 TRACK 有效标志。
    g_red_valid = 0  # 清空红点有效标志。
    g_green_valid = 0  # 清空绿点有效标志。
    g_dist_valid = 0  # 清空距离有效标志。
    g_red_x = -1  # 清空红点 X 坐标。
    g_red_y = -1  # 清空红点 Y 坐标。
    g_green_x = -1  # 清空绿点 X 坐标。
    g_green_y = -1  # 清空绿点 Y 坐标。
    g_err_x = 0  # 清空 X 像素误差。
    g_err_y = 0  # 清空 Y 像素误差。
    g_pixel_dist2 = 0  # 清空像素距离平方。
    g_track_locked = 0  # 清空临时像素阈值锁定标志。
    g_dist_mm = -1  # 清空临时等效距离毫米值。
    g_red_conf = 0  # 清空红点置信度。
    g_green_conf = 0  # 清空绿点置信度。
    g_track_err = err_code  # 记录当前错误码。

def set_track_error(err_code):  # 记录图像处理链路错误并进入可恢复错误状态。
    clear_track_result(err_code)  # 清空 TRACK 结果并记录错误码。
    enter_state(VISION_ERROR)  # 进入错误状态，等待 TI 重新发送 MODE 或 STOP。

def reset_frame_timing():  # 重置相机帧率统计，避免模式切换污染 FPS。
    global g_last_frame_ms, g_fps10  # 声明要修改上一帧时间和 FPS 观测值。
    g_last_frame_ms = 0  # 清空上一帧相机时间。
    g_fps10 = 0  # 清空 FPS10 观测值。

def reset_track_stream():  # 重置 TRACK 输出调度状态。
    global g_last_sent_frame_id, g_track_accum_ms, g_track_ready  # 声明要修改 TRACK 调度变量。
    g_last_sent_frame_id = 0  # 清空上一帧已发送帧号。
    g_track_accum_ms = config.TRACK_PERIOD_MS  # 让进入 TRACK640 后第一帧立即允许发送。
    g_track_ready = False  # 清空当前帧待发送标志。

def enter_state(next_state):  # 切换视觉状态。
    global g_state  # 声明要修改视觉状态。
    g_state = next_state  # 保存新的视觉状态。

def handle_ping(seq):  # 处理 PING 命令。
    send_ack(seq, "PING", 1, ERR_NONE)  # 回复 PING 成功。
    send_status(seq)  # 立即回一帧状态。

def handle_mode(seq, mode):  # 处理 MODE 命令。
    if mode == "TRACK640":  # 判断是否切换到双光斑模式。
        vision_spot.reset()  # 重置红点跟踪器以便重新全屏找回。
        vision_green.reset()  # 重置绿点跟踪器以便重新全屏找回。
        reset_track_stream()  # 重置 TRACK 帧率调度和相机帧统计。
        reset_frame_timing()  # 重置帧率统计以适配 640x480 双光斑模式。
        clear_track_result(ERR_NONE)  # 清空旧双光斑结果。
        enter_state(VISION_TRACK640)  # 进入双光斑坐标流状态。
        send_ack(seq, "MODE", 1, ERR_NONE)  # 回复 MODE 成功。
    else:  # 处理未知模式。
        send_ack(seq, "MODE", 0, ERR_BAD_MODE)  # 回复 MODE 失败。

def handle_stop(seq):  # 处理 STOP 命令。
    enter_state(VISION_IDLE)  # 进入空闲状态。
    clear_track_result(ERR_NONE)  # 清空旧双光斑结果。
    send_ack(seq, "STOP", 1, ERR_NONE)  # 回复 STOP 成功。

def handle_status(seq):  # 处理 STATUS 命令。
    send_status(seq)  # 发送当前状态。

def handle_command(seq, cmd, args):  # 处理 protocol.py 解析出的命令事件。
    global g_seq  # 声明要修改最近命令序号。
    if cmd == "BAD_PACKET":  # 判断 protocol.py 是否解析出坏包。
        send_err(g_seq, ERR_BAD_PACKET)  # 发送协议包错误。
        return  # 放弃本行处理。
    g_seq = seq  # 保存最近命令序号。
    if cmd == "PING":  # 判断 PING 命令。
        handle_ping(seq)  # 处理 PING。
    elif cmd == "MODE" and len(args) >= 1:  # 判断 MODE 命令。
        handle_mode(seq, args[0])  # 处理模式切换。
    elif cmd == "STOP":  # 判断 STOP 命令。
        handle_stop(seq)  # 处理停止请求。
    elif cmd == "STATUS":  # 判断 STATUS 命令。
        handle_status(seq)  # 处理状态查询。
    else:  # 处理未知命令或缺少参数的命令。
        send_ack(seq, cmd, 0, ERR_BAD_CMD)  # 回复命令失败。

def poll_uart():  # 轮询串口输入。
    events = protocol.poll_commands(g_serial)  # 从 protocol.py 取出本轮完整命令事件。
    for seq, cmd, args in events:  # 逐条处理完整命令。
        handle_command(seq, cmd, args)  # 分发命令到视觉状态机。

def update_fps(frame_start_ms):  # 根据相邻相机帧估算 FPS10。
    global g_fps10, g_last_frame_ms  # 声明要修改 FPS 和上一帧时间。
    dt = 0  # 初始化相邻帧时间间隔。
    if g_last_frame_ms > 0:  # 判断是否已有上一帧时间。
        dt = elapsed_ms(g_last_frame_ms, frame_start_ms)  # 计算两帧开始时间间隔。
        if dt > 0:  # 判断帧间隔是否合法。
            g_fps10 = int(10000 / dt)  # 计算 FPS 乘以 10 的整数值。
    g_last_frame_ms = frame_start_ms  # 保存当前帧开始时间。
    return dt  # 返回帧间隔给 TRACK 发送调度使用。

def update_track_schedule(dt_ms):  # 按 50Hz 目标从真实相机帧中挑选发送帧。
    global g_track_accum_ms, g_track_ready  # 声明要修改 TRACK 调度状态。
    if dt_ms <= 0:  # 判断是否为进入 TRACK640 后第一帧。
        g_track_ready = True  # 第一帧直接允许发送，便于 TI 快速拿到观测。
        return  # 结束第一帧调度。
    g_track_accum_ms = g_track_accum_ms + dt_ms  # 累加真实相机帧间隔。
    if g_track_accum_ms >= config.TRACK_PERIOD_MS:  # 判断累计时间是否达到目标发送周期。
        g_track_ready = True  # 标记当前相机帧需要发送。
        g_track_accum_ms = g_track_accum_ms - config.TRACK_PERIOD_MS  # 扣除一个目标发送周期。
        if g_track_accum_ms > config.TRACK_PERIOD_MS:  # 判断相机帧率低时累计时间是否过大。
            g_track_accum_ms = config.TRACK_PERIOD_MS  # 限制累计值，避免后续连续发送旧节奏。
    else:  # 处理累计时间还没到发送周期的情况。
        g_track_ready = False  # 标记当前相机帧暂不发送。

def draw_track_overlay(img):  # 绘制红绿双光斑调试叠加信息。
    if g_last_green_result:  # 判断当前是否已有绿点结果。
        vision_green.draw_overlay(img, g_last_green_result)  # 绘制绿点候选、ROI 和中心。
    if g_track_valid:  # 判断红绿双点是否同时有效。
        img.draw_line(g_green_x, g_green_y, g_red_x, g_red_y, image.COLOR_BLUE)  # 绘制从绿点到红点的像素误差线。
    mode = "TRACK"  # 生成当前显示模式字符串。
    line1 = "{} id={} fps10={} lat={} tv={}".format(mode, g_frame_id, g_fps10, g_last_latency_ms, g_track_valid)  # 生成第一行调试文本。
    line2 = "r={}({},{}) g={}({},{})".format(g_red_valid, g_red_x, g_red_y, g_green_valid, g_green_x, g_green_y)  # 生成第二行调试文本。
    line3 = "err=({},{}) d={}mm lock={}".format(g_err_x, g_err_y, g_dist_mm, g_track_locked)  # 生成第三行调试文本，d 为 10px 等效 3cm 的临时距离。
    line4 = "conf=({},{}) px3cm={}".format(g_red_conf, g_green_conf, config.TRACK_3CM_PIXEL)  # 生成第四行调试文本，显示当前 3cm 等效像素参数。
    img.draw_string(2, 2 + config.DEBUG_TEXT_STEP * 2, line1, image.COLOR_GREEN, scale=config.DEBUG_TEXT_SCALE)  # 避开红点模块前两行文字绘制双点状态。
    img.draw_string(2, 2 + config.DEBUG_TEXT_STEP * 3, line2, image.COLOR_GREEN, scale=config.DEBUG_TEXT_SCALE)  # 避开红点模块文字绘制红绿坐标。
    img.draw_string(2, 2 + config.DEBUG_TEXT_STEP * 4, line3, image.COLOR_GREEN, scale=config.DEBUG_TEXT_SCALE)  # 避开红点模块文字绘制像素误差和置信度。
    img.draw_string(2, 2 + config.DEBUG_TEXT_STEP * 5, line4, image.COLOR_GREEN, scale=config.DEBUG_TEXT_SCALE)  # 绘制临时等效像素参数和红绿置信度。

def update_track_frame(frame_start_ms):  # 读取并处理一帧红绿双光斑图像。
    global g_frame_id  # 声明要修改帧号。
    setup_camera()  # 确保固定 640x480 相机和显示已经初始化。
    img = read_camera_frame()  # 安全读取一帧 640x480 RGB 图像。
    if img is None:  # 判断相机读取是否失败。
        return  # 读取失败时结束本帧处理并等待 TI 重新下发模式。
    g_frame_id = g_frame_id + 1  # 增加真实相机处理帧号。
    red_result = vision_spot.detect(img)  # 检测红色激光点，算法沿用红色工程。
    green_result = vision_green.detect(img)  # 检测绿色激光点。
    latency_ms = elapsed_ms(frame_start_ms, now_ms())  # 记录采集加双点识别耗时。
    dt_ms = update_fps(frame_start_ms)  # 更新 FPS10 并取得相邻帧间隔。
    update_track_schedule(dt_ms)  # 根据真实帧间隔更新 TRACK 发送调度。
    set_track_result(red_result, green_result, latency_ms)  # 更新协议侧红绿双点结果。
    if g_frame_id % config.DEBUG_RENDER_EVERY == 0:  # 判断是否到达调试画面刷新帧。
        vision_spot.draw_overlay(img, red_result, g_frame_id, g_fps10, g_last_latency_ms)  # 绘制红点检测调试信息。
        draw_track_overlay(img)  # 绘制绿点、误差线和双点汇总调试信息。
        g_display.show(img)  # 显示调试画面。

def poll_outputs(current_ms):  # 根据当前状态发送周期输出。
    global g_last_status_ms, g_last_track_ms, g_last_sent_frame_id, g_track_ready  # 声明要修改周期输出变量。
    if elapsed_ms(g_last_status_ms, current_ms) >= config.STATUS_PERIOD_MS:  # 判断是否到达状态输出周期。
        g_last_status_ms = current_ms  # 更新状态输出时间。
        send_status(g_seq)  # 发送低频状态包。
    if g_state == VISION_TRACK640 and g_track_ready and g_frame_id != g_last_sent_frame_id:  # 判断是否有新相机帧需要发送 TRACK。
        g_last_track_ms = current_ms  # 更新 TRACK 输出时间。
        g_last_sent_frame_id = g_frame_id  # 记录本次已发送的相机帧号。
        g_track_ready = False  # 清除当前帧待发送标志。
        send_track(g_seq)  # 发送当前红绿双点结果。

def main():  # MaixCAM2 通信主入口。
    global g_serial, g_last_track_ms, g_last_status_ms  # 声明要修改主循环全局变量。
    print("GREEN_VISION_APP UART2 {} {} {}".format(config.UART_TX_PIN, config.UART_RX_PIN, config.UART_DEVICE))  # 输出自定义启动标识，区分系统日志。
    g_serial = setup_uart()  # 初始化并打开串口。
    start_ms = now_ms()  # 读取启动时间。
    g_last_track_ms = start_ms  # 初始化 TRACK 时间。
    g_last_status_ms = start_ms  # 初始化 STATUS 时间。
    send_status(g_seq)  # 启动后发送一帧初始状态。
    while not app.need_exit():  # 持续运行直到 MaixPy 应用退出。
        loop_start_ms = now_ms()  # 记录本轮循环开始时间。
        poll_uart()  # 轮询并处理 TI 命令。
        if g_state == VISION_TRACK640:  # 判断当前是否需要运行双光斑识别。
            update_track_frame(loop_start_ms)  # 读取相机并更新红绿双点结果。
        poll_outputs(now_ms())  # 按状态输出协议包。

main()  # 启动通信主程序。
