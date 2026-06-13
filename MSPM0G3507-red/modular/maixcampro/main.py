from maix import app, camera, display, err, pinmap, time, uart  # 导入 MaixPy 运行、相机、显示、错误、引脚、时间和串口模块。
import config  # 导入现场可调参数配置模块。
import protocol  # 导入项目通信协议封包、解包和串口轮询模块。
import vision_a4  # 导入 A4 黑胶带闭合框检测模块。
import vision_spot  # 导入红色激光点检测模块。

VISION_IDLE = "VISION_IDLE"  # 定义视觉空闲状态。
VISION_SPOT640 = "VISION_SPOT640"  # 定义 640x480 RGB888 红点检测状态。
VISION_A4GRAY = "VISION_A4GRAY"  # 定义 640x480 A4 黑框检测状态。
VISION_A4_LOCKED = "VISION_A4_LOCKED"  # 定义 A4 结果锁存交付状态。
VISION_ERROR = "VISION_ERROR"  # 定义视觉错误状态。

ERR_NONE = 0  # 定义无错误码。
ERR_BAD_PACKET = 1  # 定义协议包格式错误码。
ERR_BAD_CMD = 2  # 定义未知命令错误码。
ERR_BAD_MODE = 3  # 定义未知模式错误码。
ERR_A4_NOT_READY = 4  # 定义 A4 当前不可锁存错误码。
ERR_LOCK_TIMEOUT = 5  # 定义 A4 锁存结果重发超时错误码。
ERR_A4_PROCESS = 6  # 定义 A4 图像处理不可用或异常错误码。

g_serial = None  # 保存串口对象。
g_camera = None  # 保存相机对象。
g_display = None  # 保存显示对象。
g_state = VISION_SPOT640  # 保存当前视觉状态。
g_seq = "0"  # 保存最近一次 TI 命令序号。
g_frame_id = 0  # 保存真实处理过的相机帧号。
g_fps10 = 0  # 保存视觉处理 FPS 乘以 10 的整数值。
g_last_loop_ms = 0  # 保存上一轮循环时间。
g_last_frame_ms = 0  # 保存上一帧相机处理开始时间。
g_last_latency_ms = 0  # 保存上一帧采集加识别耗时。
g_last_spot_ms = 0  # 保存上一帧 SPOT 发送时间。
g_last_sent_frame_id = 0  # 保存上一帧已发送的相机帧号。
g_spot_accum_ms = 0  # 保存 SPOT 目标 50Hz 发送调度累计时间。
g_spot_ready = False  # 保存当前相机帧是否应该发送 SPOT。
g_last_status_ms = 0  # 保存上一帧 STATUS 发送时间。
g_last_locked_ms = 0  # 保存上一帧 A4_LOCKED 发送时间。
g_locked_retry_count = 0  # 保存 A4_LOCKED 已重发次数。
g_locked_active = False  # 保存 A4_LOCKED 是否正在等待 TI ACK。
g_spot_valid = 0  # 保存红点检测有效标志。
g_spot_x = -1  # 保存红点 X 像素坐标。
g_spot_y = -1  # 保存红点 Y 像素坐标。
g_spot_conf = 0  # 保存红点置信度。
g_spot_err = ERR_NONE  # 保存红点检测错误码。
g_a4_valid = 0  # 保存 A4 检测有效标志。
g_a4_points = [0, 0, 0, 0, 0, 0, 0, 0]  # 保存 A4 中心线四角点坐标。
g_a4_angle10 = 0  # 保存 A4 角度，单位为 0.1 度。
g_a4_conf = 0  # 保存 A4 检测置信度。
g_a4_err = ERR_NONE  # 保存 A4 检测错误码。
g_locked_a4_overlay_points = None  # 保存已锁存 A4 红框 overlay，直到下一次 A4 检测开始。

def now_ms():  # 获取当前毫秒时间。
    return time.ticks_ms()  # 返回 MaixPy 毫秒计数。

def elapsed_ms(start_ms, current_ms):  # 计算两个毫秒时间之间的差值。
    return current_ms - start_ms  # 返回当前时间减起始时间。

def setup_uart():  # 初始化 MaixCAM-Pro UART1。
    err.check_raise(pinmap.set_pin_function(config.UART_TX_PIN, config.UART_TX_FUNC), "set UART TX failed")  # 按配置复用 TX 引脚。
    err.check_raise(pinmap.set_pin_function(config.UART_RX_PIN, config.UART_RX_FUNC), "set UART RX failed")  # 按配置复用 RX 引脚。
    return uart.UART(config.UART_DEVICE, config.UART_BAUD)  # 按配置打开 UART 串口对象。

def set_process_error(err_code):  # 记录图像处理链路错误并进入可恢复错误状态。
    set_spot_result(0, -1, -1, 0, err_code)  # 清空红点结果并记录错误码。
    set_a4_result(0, [0, 0, 0, 0, 0, 0, 0, 0], 0, 0, err_code)  # 清空 A4 结果并记录错误码。
    enter_state(VISION_ERROR)  # 进入错误状态，等待 TI 重新发送 MODE 或 STOP。

def current_error():  # 获取当前状态错误码。
    if g_state == VISION_ERROR:  # 判断当前是否处于错误状态。
        return g_a4_err if g_a4_err != ERR_NONE else g_spot_err  # 优先返回 A4 错误，否则返回红点错误。
    return ERR_NONE  # 非错误状态返回无错误。

def send_ack(seq, cmd, ok, err_code):  # 发送命令确认包。
    protocol.send_ack(g_serial, seq, cmd, ok, err_code)  # 通过 protocol.py 输出 ACK,cmd,ok,err。

def send_err(seq, err_code):  # 发送视觉错误包。
    protocol.send_err(g_serial, seq, err_code)  # 通过 protocol.py 输出 ERR,code。

def send_status(seq):  # 发送当前视觉状态包。
    protocol.send_status(g_serial, seq, g_state, g_fps10, g_frame_id, current_error())  # 通过 protocol.py 输出 STATUS,mode,fps10,frame,err。

def send_spot(seq):  # 发送红点坐标包。
    protocol.send_spot(g_serial, seq, g_frame_id, g_spot_valid, g_spot_x, g_spot_y, g_spot_conf, g_last_latency_ms, g_spot_err)  # 通过 protocol.py 输出 SPOT 字段。

def send_a4_locked(seq):  # 发送 A4 锁存结果包。
    protocol.send_a4_locked(g_serial, seq, g_frame_id, g_a4_points, g_a4_angle10, g_a4_conf, g_a4_err)  # 通过 protocol.py 输出 A4_LOCKED 字段。

def set_spot_result(valid, x, y, conf, err_code):  # 给后续红点算法写入结果。
    global g_spot_valid, g_spot_x, g_spot_y, g_spot_conf, g_spot_err  # 声明要修改红点全局结果。
    g_spot_valid = valid  # 更新红点有效标志。
    g_spot_x = x  # 更新红点 X 坐标。
    g_spot_y = y  # 更新红点 Y 坐标。
    g_spot_conf = conf  # 更新红点置信度。
    g_spot_err = err_code  # 更新红点错误码。

def set_a4_result(valid, points, angle10, conf, err_code):  # 给后续 A4 算法写入结果。
    global g_a4_valid, g_a4_points, g_a4_angle10, g_a4_conf, g_a4_err  # 声明要修改 A4 全局结果。
    g_a4_valid = valid  # 更新 A4 有效标志。
    g_a4_points = points  # 更新 A4 四角点坐标。
    g_a4_angle10 = angle10  # 更新 A4 角度。
    g_a4_conf = conf  # 更新 A4 置信度。
    g_a4_err = err_code  # 更新 A4 错误码。

def reset_a4_result():  # 清空 A4 检测结果，避免模式切换后锁存旧角点。
    set_a4_result(0, [0, 0, 0, 0, 0, 0, 0, 0], 0, 0, ERR_NONE)  # 写入无效 A4 结果。

def save_locked_a4_overlay():  # 保存当前锁存 A4 红框用于后续显示保留。
    global g_locked_a4_overlay_points  # 声明要修改锁存红框 overlay 角点。
    g_locked_a4_overlay_points = list(g_a4_points)  # 复制当前稳定 A4 中心线角点，避免后续结果清空影响显示。

def clear_locked_a4_overlay():  # 清除旧的锁存 A4 红框 overlay。
    global g_locked_a4_overlay_points  # 声明要修改锁存红框 overlay 角点。
    g_locked_a4_overlay_points = None  # 下一次黑框检测开始时清除旧红框。

def draw_locked_a4_overlay(img):  # 在当前画面上叠加已锁存 A4 红框。
    if g_locked_a4_overlay_points is None:  # 判断当前是否没有需要保留的锁存红框。
        return  # 没有锁存红框时不绘制。
    vision_a4.draw_locked_overlay(img, g_locked_a4_overlay_points)  # 绘制上一次锁存的 A4 中心线红框。

def reset_frame_timing():  # 重置相机帧率统计，避免模式切换污染 FPS。
    global g_last_frame_ms, g_fps10  # 声明要修改上一帧时间和 FPS 观测值。
    g_last_frame_ms = 0  # 清空上一帧相机时间。
    g_fps10 = 0  # 清空 FPS10 观测值。

def enter_state(next_state):  # 切换视觉状态。
    global g_state, g_locked_active, g_locked_retry_count  # 声明要修改状态和锁存重发变量。
    g_state = next_state  # 保存新的视觉状态。
    if next_state != VISION_A4_LOCKED:  # 判断是否离开 A4 锁存交付状态。
        g_locked_active = False  # 离开锁存状态时关闭重发。
        g_locked_retry_count = 0  # 离开锁存状态时清零重发计数。

def handle_ping(seq):  # 处理 PING 命令。
    send_ack(seq, "PING", 1, ERR_NONE)  # 回复 PING 成功。
    send_status(seq)  # 立即回一帧状态。

def handle_mode(seq, mode):  # 处理 MODE 命令。
    if mode == "SPOT640":  # 判断是否切换到红点模式。
        vision_spot.reset()  # 重置红点跟踪器以便重新全屏找回。
        reset_spot_stream()  # 重置 SPOT 帧率调度和相机帧统计。
        reset_frame_timing()  # 重置帧率统计以适配 640x480 红点模式。
        reset_a4_result()  # 清空旧 A4 结果以免后续误锁存。
        enter_state(VISION_SPOT640)  # 进入红点坐标流状态。
        send_ack(seq, "MODE", 1, ERR_NONE)  # 回复 MODE 成功。
    elif mode == "A4GRAY":  # 判断是否切换到 A4 灰度模式。
        clear_locked_a4_overlay()  # 开始下一次黑框检测时清除上一次锁存红框。
        vision_a4.reset()  # 重置 A4 多帧稳定判定。
        reset_a4_result()  # 清空旧 A4 角点和置信度。
        reset_frame_timing()  # 重置帧率统计以适配 640x480 A4 模式。
        enter_state(VISION_A4GRAY)  # 进入 A4 灰度检测状态。
        send_ack(seq, "MODE", 1, ERR_NONE)  # 回复 MODE 成功。
    else:  # 处理未知模式。
        send_ack(seq, "MODE", 0, ERR_BAD_MODE)  # 回复 MODE 失败。

def handle_lock_a4(seq):  # 处理 LOCK_A4 命令。
    global g_last_locked_ms, g_locked_retry_count, g_locked_active  # 声明要修改 A4 锁存重发变量。
    if g_state != VISION_A4GRAY:  # 判断当前是否不在 A4 检测状态。
        send_ack(seq, "LOCK_A4", 0, ERR_BAD_MODE)  # 回复当前模式不允许锁存。
        return  # 结束 LOCK_A4 处理。
    if g_a4_valid == 0:  # 判断当前是否没有稳定 A4 结果。
        send_ack(seq, "LOCK_A4", 0, ERR_A4_NOT_READY)  # 回复 A4 当前不可锁存。
        send_err(seq, ERR_A4_NOT_READY)  # 同步发送错误码。
        return  # 结束 LOCK_A4 处理。
    save_locked_a4_overlay()  # 保存锁存红框，供后续红点画面持续显示到下一次黑框检测。
    enter_state(VISION_A4_LOCKED)  # 进入 A4 锁存交付状态。
    g_locked_active = True  # 打开 A4_LOCKED 重发等待。
    g_locked_retry_count = 0  # 清零 A4_LOCKED 重发次数。
    g_last_locked_ms = now_ms()  # 记录 A4_LOCKED 发送时间。
    send_a4_locked(seq)  # 发送首帧 A4_LOCKED。

def handle_ack(seq, target):  # 处理 TI 的 ACK 命令。
    if target == "A4_LOCKED":  # 判断 TI 是否确认 A4_LOCKED。
        enter_state(VISION_IDLE)  # 停止继续检测黑框，等待 TI 后续 MODE 命令。
        send_ack(seq, "ACK", 1, ERR_NONE)  # 回复 ACK 命令成功。
    else:  # 处理未知 ACK 目标。
        send_ack(seq, "ACK", 0, ERR_BAD_CMD)  # 回复 ACK 命令失败。

def handle_stop(seq):  # 处理 STOP 命令。
    enter_state(VISION_IDLE)  # 进入空闲状态。
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
    elif cmd == "LOCK_A4":  # 判断 LOCK_A4 命令。
        handle_lock_a4(seq)  # 处理 A4 锁存请求。
    elif cmd == "STOP":  # 判断 STOP 命令。
        handle_stop(seq)  # 处理停止请求。
    elif cmd == "STATUS":  # 判断 STATUS 命令。
        handle_status(seq)  # 处理状态查询。
    elif cmd == "ACK" and len(args) >= 1:  # 判断 ACK 命令。
        handle_ack(seq, args[0])  # 处理 ACK 目标。
    else:  # 处理未知命令或缺少参数的命令。
        send_ack(seq, cmd, 0, ERR_BAD_CMD)  # 回复命令失败。

def poll_uart():  # 轮询串口输入。
    events = protocol.poll_commands(g_serial)  # 从 protocol.py 取出本轮完整命令事件。
    for seq, cmd, args in events:  # 逐条处理完整命令。
        handle_command(seq, cmd, args)  # 分发命令到视觉状态机。

def setup_camera():  # 初始化固定 640x480 相机和显示。
    global g_camera, g_display  # 声明要修改相机和显示对象。
    if not g_camera:  # 判断相机是否尚未初始化。
        g_camera = camera.Camera(config.SPOT_IMG_W, config.SPOT_IMG_H, fps=config.CAMERA_FPS)  # 固定使用 640x480 相机，避免模式切换重建相机导致读帧超时。
    if not g_display:  # 判断显示是否尚未初始化。
        g_display = display.Display()  # 创建 MaixCAM 显示对象。

def read_camera_frame():  # 安全读取一帧相机图像。
    try:  # 捕获板端 camera.read 超时或底层返回空指针异常。
        return g_camera.read()  # 读取一帧固定 640x480 图像。
    except Exception:  # 处理相机读取失败，避免主程序退出。
        set_process_error(ERR_A4_PROCESS)  # 记录处理错误并进入可恢复错误状态。
        return None  # 返回空图像让当前帧处理提前结束。

def reset_spot_stream():  # 重置 SPOT 输出调度状态。
    global g_last_frame_ms, g_last_sent_frame_id, g_spot_accum_ms, g_spot_ready  # 声明要修改 SPOT 调度变量。
    g_last_sent_frame_id = 0  # 清空上一帧已发送帧号。
    g_spot_accum_ms = config.SPOT_PERIOD_MS  # 让进入 SPOT640 后第一帧立即允许发送。
    g_spot_ready = False  # 清空当前帧待发送标志。

def update_fps(frame_start_ms):  # 根据相邻相机帧估算 FPS10。
    global g_fps10, g_last_frame_ms  # 声明要修改 FPS 和上一帧时间。
    dt = 0  # 初始化相邻帧时间间隔。
    if g_last_frame_ms > 0:  # 判断是否已有上一帧时间。
        dt = elapsed_ms(g_last_frame_ms, frame_start_ms)  # 计算两帧开始时间间隔。
        if dt > 0:  # 判断帧间隔是否合法。
            g_fps10 = int(10000 / dt)  # 计算 FPS 乘以 10 的整数值。
    g_last_frame_ms = frame_start_ms  # 保存当前帧开始时间。
    return dt  # 返回帧间隔给 SPOT 发送调度使用。

def update_spot_schedule(dt_ms):  # 按 50Hz 目标从真实相机帧中挑选发送帧。
    global g_spot_accum_ms, g_spot_ready  # 声明要修改 SPOT 调度状态。
    if dt_ms <= 0:  # 判断是否为进入 SPOT640 后第一帧。
        g_spot_ready = True  # 第一帧直接允许发送，便于 TI 快速拿到坐标。
        return  # 结束第一帧调度。
    g_spot_accum_ms = g_spot_accum_ms + dt_ms  # 累加真实相机帧间隔。
    if g_spot_accum_ms >= config.SPOT_PERIOD_MS:  # 判断累计时间是否达到目标发送周期。
        g_spot_ready = True  # 标记当前相机帧需要发送。
        g_spot_accum_ms = g_spot_accum_ms - config.SPOT_PERIOD_MS  # 扣除一个目标发送周期。
        if g_spot_accum_ms > config.SPOT_PERIOD_MS:  # 判断相机帧率低时累计时间是否过大。
            g_spot_accum_ms = config.SPOT_PERIOD_MS  # 限制累计值，避免后续连续发送旧节奏。
    else:  # 处理累计时间还没到发送周期的情况。
        g_spot_ready = False  # 标记当前相机帧暂不发送。

def update_spot_frame(frame_start_ms):  # 读取并处理一帧红点图像。
    global g_frame_id, g_last_latency_ms  # 声明要修改帧号和处理耗时。
    setup_camera()  # 确保固定 640x480 相机和显示已经初始化。
    img = read_camera_frame()  # 安全读取一帧 640x480 RGB 图像。
    if img is None:  # 判断相机读取是否失败。
        return  # 读取失败时结束本帧处理并等待 TI 重新下发模式。
    g_frame_id = g_frame_id + 1  # 增加真实相机处理帧号。
    result = vision_spot.detect(img)  # 检测红色激光点。
    g_last_latency_ms = elapsed_ms(frame_start_ms, now_ms())  # 记录采集加识别耗时。
    dt_ms = update_fps(frame_start_ms)  # 更新 FPS10 并取得相邻帧间隔。
    update_spot_schedule(dt_ms)  # 根据真实帧间隔更新 SPOT 发送调度。
    set_spot_result(result["valid"], result["x"], result["y"], result["conf"], result["err"])  # 更新协议侧红点结果。
    if g_frame_id % config.DEBUG_RENDER_EVERY == 0:  # 判断是否到达调试画面刷新帧。
        vision_spot.draw_overlay(img, result, g_frame_id, g_fps10, g_last_latency_ms)  # 绘制红点检测调试信息。
        draw_locked_a4_overlay(img)  # 在红点画面上保留上一次锁存的 A4 红框。
        g_display.show(img)  # 显示调试画面。

def update_a4_frame(frame_start_ms):  # 读取并处理一帧 A4 黑框图像。
    global g_frame_id, g_last_latency_ms  # 声明要修改帧号和处理耗时。
    setup_camera()  # 确保固定 640x480 相机和显示已经初始化。
    img = read_camera_frame()  # 安全读取一帧 640x480 图像。
    if img is None:  # 判断相机读取是否失败。
        return  # 读取失败时结束本帧处理并等待 TI 重新下发模式。
    g_frame_id = g_frame_id + 1  # 增加真实相机处理帧号。
    result = vision_a4.detect(img)  # 检测 A4 黑色胶带闭合框。
    g_last_latency_ms = elapsed_ms(frame_start_ms, now_ms())  # 记录采集加识别耗时。
    update_fps(frame_start_ms)  # 更新 FPS10 观测值。
    set_a4_result(result["valid"], result["points"], result["angle10"], result["conf"], result["err"])  # 更新协议侧 A4 锁存候选结果。
    if g_frame_id % config.DEBUG_RENDER_EVERY == 0:  # 判断是否到达调试画面刷新帧。
        vision_a4.draw_overlay(img, result, g_frame_id, g_fps10, g_last_latency_ms)  # 绘制 A4 检测调试信息。
        g_display.show(img)  # 显示 A4 调试画面。

def poll_outputs(current_ms):  # 根据当前状态发送周期输出。
    global g_last_spot_ms, g_last_sent_frame_id, g_spot_ready, g_last_status_ms, g_last_locked_ms, g_locked_retry_count  # 声明要修改周期输出变量。
    if elapsed_ms(g_last_status_ms, current_ms) >= config.STATUS_PERIOD_MS:  # 判断是否到达状态输出周期。
        g_last_status_ms = current_ms  # 更新状态输出时间。
        send_status(g_seq)  # 发送低频状态包。
    if g_state == VISION_SPOT640 and g_spot_ready and g_frame_id != g_last_sent_frame_id:  # 判断是否有新相机帧需要发送 SPOT。
        g_last_spot_ms = current_ms  # 更新 SPOT 输出时间。
        g_last_sent_frame_id = g_frame_id  # 记录本次已发送的相机帧号。
        g_spot_ready = False  # 清除当前帧待发送标志。
        send_spot(g_seq)  # 发送当前红点结果。
    if g_locked_active and elapsed_ms(g_last_locked_ms, current_ms) >= config.A4_LOCKED_RESEND_MS:  # 判断是否需要重发 A4_LOCKED。
        if g_locked_retry_count >= config.A4_LOCKED_MAX_RETRY:  # 判断是否已经超过最大重发次数。
            set_a4_result(g_a4_valid, g_a4_points, g_a4_angle10, g_a4_conf, ERR_LOCK_TIMEOUT)  # 记录锁存超时错误，确保 STATUS 能带出 err=5。
            enter_state(VISION_ERROR)  # 进入视觉错误状态。
            send_err(g_seq, ERR_LOCK_TIMEOUT)  # 发送锁存超时错误。
        else:  # 处理仍可重发的情况。
            g_locked_retry_count = g_locked_retry_count + 1  # 增加 A4_LOCKED 重发次数。
            g_last_locked_ms = current_ms  # 更新 A4_LOCKED 重发时间。
            send_a4_locked(g_seq)  # 重发 A4_LOCKED。

def main():  # MaixCAM-Pro 通信主入口。
    global g_serial, g_last_loop_ms, g_last_spot_ms, g_last_status_ms  # 声明要修改主循环全局变量。
    print("RED_VISION_APP UART1 A19/A18 /dev/ttyS1")  # 输出自定义启动标识，区分 MaixVision 内部 UART0 日志。
    g_serial = setup_uart()  # 初始化并打开串口。
    g_last_loop_ms = now_ms()  # 初始化循环时间。
    g_last_spot_ms = g_last_loop_ms  # 初始化 SPOT 时间。
    g_last_status_ms = g_last_loop_ms  # 初始化 STATUS 时间。
    send_status(g_seq)  # 启动后发送一帧初始状态。
    while not app.need_exit():  # 持续运行直到 MaixPy 应用退出。
        loop_start_ms = now_ms()  # 记录本轮循环开始时间。
        poll_uart()  # 轮询并处理 TI 命令。
        if g_state == VISION_SPOT640:  # 判断当前是否需要运行红点识别。
            update_spot_frame(loop_start_ms)  # 读取相机并更新红点结果。
        elif g_state == VISION_A4GRAY:  # 判断当前是否需要运行 A4 黑框识别。
            update_a4_frame(loop_start_ms)  # 读取相机并更新 A4 锁存候选结果。
        poll_outputs(now_ms())  # 按状态输出协议包。

main()  # 启动通信主程序。
