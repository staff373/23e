# 项目地图

## 目标
- 当前目标：准备 2023 全国大学生电子设计竞赛本科组 E 题“运动目标控制与自动追踪系统”。
- 系统由两个相互独立的光斑位置控制系统组成：红色光斑模拟运动目标，绿色光斑用于自动追踪红色光斑。
- 红色、绿色系统之间不得有任何方式通信；屏幕上不能放电子元件；不能使用台式机或笔记本电脑作为控制系统。
- 近期工程目标：只专注红色激光系统，先建立可实现基本要求（1）-（4）的红色运动目标控制和 MaixCAM-Pro 视觉状态机。

## 硬件
- 红色系统主控：TI 端，使用 `MSPM0G3507`；当前固件工程位于 `firmware/mspm0g3507-red`，由 CCS/Theia 空工程迁移而来，SysConfig 生成文件位于 `Debug/`。
- 红色系统视觉：`MaixCAM-Pro`，通过 UART 接收 TI 命令并输出红点坐标流、A4 锁存结果和视觉状态。
- 按键：红色系统当前有 3 个按键，均接在 TI 端；当前约定 `K1` 短按暂停/放弃、`K1` 长按复位到原点，`K2` 启动/保存，`K3` 切模式/切换选择项。
- 执行器：两个独立二维电控云台，各固定一支激光笔。
- 光源约束：红色、绿色光斑直径均需小于或等于 1cm。
- 屏幕：白色，有效面积大于 `0.6m x 0.6m`；中心画 `0.5m x 0.5m` 正方形边线和原点。
- 摆放：红色激光笔正对屏幕，距离约 1m；绿色激光笔在红色激光笔两侧放置线段上任意放置，线段与屏幕平行，距红色激光笔大于 0.4m、小于 1m。
- 外设归属：`empty.syscfg` 负责 SYSCTL、按键 GPIO、按键反馈 LED 和 PB22 运行灯生成；`modular/bsp_vision_uart` 负责 `UART1 PA8/PA9` 视觉串口；`modular/bsp_bt` 负责 `UART3 PA26/PA25` 蓝牙串口；`modular/bsp_stepper` 负责 X/Y 两轴 STEP/DIR/EN；`modular/bsp_oled` 负责软件 I2C `PA4/PA5`；`modular/bsp_jy61p` 负责 `UART0 PA10/PA11`，当前默认不启动；直流电机 BSP 已迁入但默认不启动。

## 代码地图
- `modular/maixcampro/`：MaixCAM-Pro 视觉侧 MaixPy 应用；当前包含 `main.py`、`vision_spot.py`、`vision_a4.py`、`protocol.py`、`config.py`、`app.yaml` 和 `dist/maix-red_vision-v0.1.0.zip`。
- `modular/maixcampro/main.py`：MaixCAM 运行入口、视觉状态机、UART 命令分发、周期 `STATUS/SPOT/A4_LOCKED` 输出。
- `modular/maixcampro/vision_spot.py`：红色激光点传统视觉检测；使用 LAB 阈值 `find_blobs`、上一帧 ROI 跟踪、候选过滤和白芯亮度加权精定位。
- `modular/maixcampro/vision_a4.py`：A4 黑胶带闭合框传统视觉检测；使用 OpenCV 黑色二值化、轮廓层级、外框/内孔四边形拟合、边线精修和多帧稳定判定输出中心线四角点。
- `modular/maixcampro/protocol.py`：TI-MaixCAM ASCII CSV 协议封包、解包和非阻塞串口轮询。
- `modular/maixcampro/config.py`：MaixCAM UART、红点帧率、阈值、ROI、候选过滤、白芯精定位参数和 A4 黑框几何过滤参数。
- TI/MSPM0G3507 工程：`firmware/mspm0g3507-red/empty.c` 为当前入口；`modular/` 已迁入按键、按键 LED、蓝牙 UART3、视觉 UART1、视觉通信、步进、OLED、JY61P 和直流电机 BSP/APP 初始化依赖。当前尚未接入 `app_red_task` 主状态机。
- `docs/state_machines.html`：红色系统 TI 主控主状态机、完整状态转换关系、标定保存内容和 TI-MaixCAM 通信链路；当前仍是 `preview` 状态预览。
- `2023_E_2023 E题 运动目标控制与自动追踪系统.pdf`：当前题目资料来源。
- 建议分层：`app_red_task` 负责 TI 主控主状态机、模式选择、按键、暂停、`K1` 长按复位、标定保存、A4 检测等待和错误出口；运动控制子模块负责轨迹和执行器输出；`vision_app` 负责 MaixCAM-Pro 红点检测、A4 灰度检测和 UART 结果输出；BSP/driver 层只做硬件抽象，不承载比赛业务逻辑。

## 运行路径
- TI Init order：当前最小运行版为 `SYSCFG_DL_init()`、`SysTick 1ms`、按键/按键 LED、PB22 运行灯、蓝牙 UART3、视觉通信 UART1、步进 BSP；OLED、JY61P、直流电机保留编译和可开关入口但默认关闭。
- TI Main loop/task order：当前轮询按键锁存显示、PB22 运行灯、`VisionComm_Poll()`；JY61P 轮询默认关闭。按键仍使用已验证的“单键锁存到全松手”逻辑，尚未接入长短按事件到 `app_red_task`。
- MaixCAM Init order：`main.py` 启动后打印启动标识，复用 A19/A18 为 UART1，打开 `/dev/ttyS1`，初始化时间戳，并发送一帧 `STATUS`。
- MaixCAM Main loop order：轮询 UART 命令；`VISION_SPOT640` 和 `VISION_A4GRAY` 都读取固定 640x480 相机帧；红点模式直接输出 640x480 坐标并可叠加上一次锁存 A4 红框，A4 测试版直接在 640x480 上更新 A4 锁存候选；最后按周期输出 `STATUS`、`SPOT` 或锁存重发包。
- MaixCAM camera/display：当前固定使用 `camera.Camera(640, 480, fps=60)` 和 `display.Display()`；不在 `SPOT640/A4GRAY` 模式切换时重建相机，避免板端 MMF 重新初始化后读帧超时。
- 重要 callback/interrupt dispatch：TBD，待 TI 固件工程建立；MaixCAM 当前使用主循环轮询，不承载运动控制中断逻辑。

## 状态机总览
- 详表：`docs/state_machines.html`。
- `app_red_task`：TI 主控主状态机，当前状态为 `BOOT/CALIB_SCREEN/IDLE/RESET_RUN/SQUARE_RUN/A4_DETECT/A4_READY/A4_RUN/PAUSED/DONE/ERROR`；`RESET_RUN` 由 `K1` 长按触发，不作为 `mode` 选项；关键 STATUS 字段为 `state/prev_state/mode/calib_valid/saved/pt/spot_valid/spot/a4_valid/path/target/err`。
- `vision_app`：MaixCAM-Pro 侧维护 `VISION_IDLE/VISION_SPOT640/VISION_A4GRAY/VISION_A4_LOCKED/VISION_ERROR`；当前启动默认进入 `VISION_SPOT640`。
- `VISION_SPOT640`：已实现红点坐标流，输出 `SPOT,frame,640,480,valid,x,y,conf,lat,err`；板端 overlay 显示 `frame_id/fps10/latency/valid/conf/core/ROI`。
- `VISION_A4GRAY`：已接入 A4 黑胶带闭合框检测测试版；固定采集 640x480，直接在 640x480 上通过黑色二值化、外框/内孔轮廓、边线精修和多帧稳定判定更新 `g_a4_valid/g_a4_points/g_a4_angle10/g_a4_conf/g_a4_err`。
- `VISION_A4_LOCKED`：锁存重发和 `ACK,A4_LOCKED` 回收逻辑已实现；确认后进入 `VISION_IDLE` 等待 TI 后续 `MODE,SPOT640`，已锁存红框保留到下一次 `MODE,A4GRAY`；A4 候选未稳定时，`LOCK_A4` 会返回 `ERR_A4_NOT_READY`。

## 接入规则
- 推荐把比赛行为放在 app 层状态机：启动标定、复位、跑边线、跑 A4 靶纸、旋转靶纸、追踪、暂停、声光提示都应有明确状态和超时条件。
- 红色系统和绿色系统必须在架构上保持独立；不得通过 UART、共享变量、共享总线协议或软件消息互相传递红色位置。
- 红色系统内部 TI 与 MaixCAM-Pro 可以通信；TI 是运动控制 owner，MaixCAM-Pro 是视觉检测 owner。
- TI 端按键处理只能调用 `app_red_task` 公开 API，不应直接改运动控制或视觉私有状态。
- 屏幕标定保存 `P0~P4`：`P0` 为原点，`P1~P4` 为第二问 `0.5m x 0.5m` 正方形四角点；顺序为原点、左上、右上、右下、左下；标定时电机失能，人工移动光点，TI 在 `K2` 时保存最近有效 `SPOT` 坐标。
- MaixCAM-Pro 不直接控制云台/电机/舵机；只输出视觉结果、自身状态和错误原因；复位的云台运动链路归 TI 和运动控制模块所有。
- 不推荐把路径规划、追踪判定、暂停策略直接写进中断、定时器 callback、GPIO callback 或底层 PWM driver。
- 不推荐为同一外设重复创建初始化代码；工程建立后必须记录唯一外设所有者。

## TI-MaixCAM 通信协议
- 物理层：`3.3V TTL UART`，共地，默认 `115200 8N1`；MaixCAM 当前配置为 `/dev/ttyS1`，A19=`UART1_TX`，A18=`UART1_RX`。
- 包格式：ASCII CSV，一行一包，`\r\n` 结尾；`#` 开头为视觉调试行，TI 忽略。
- 方向：TI 是命令方和状态机 owner；MaixCAM-Pro 是视觉检测 owner；标定点和 A4 锁存结果由 TI 保存或锁存，MaixCAM 不长期重发业务结果。
- 命令格式：`>T,<seq>,<cmd>[,<arg>...]`；回包格式：`<C,<seq>,<type>[,<field>...]`；`seq` 由 TI 递增，一次性结果必须匹配当前请求。
- TI 命令：`PING`、`MODE,SPOT640`、`MODE,A4GRAY`、`LOCK_A4`、`STOP`、`STATUS`、`ACK,A4_LOCKED`。
- MaixCAM 回包：`ACK,cmd,ok,err`、`STATUS,mode,fps10,frame,err`、`SPOT,frame,iw,ih,valid,x,y,conf,lat,err`、`A4_LOCKED,frame,iw,ih,x0,y0,x1,y1,x2,y2,x3,y3,angle10,conf,err`、`ERR,code`。
- `SPOT640`：MaixCAM 使用 `640x480 RGB888`，长期发送当前红色激光点像素坐标；标定、复位、正方形和 A4 运行阶段均可使用。
- `A4GRAY`：固定采集 640x480，A4 测试版直接在 `640x480` 上检测 A4 黑色电工胶带闭合框；稳定前 `LOCK_A4` 返回 `ERR_A4_NOT_READY`，稳定后返回中心线四角点、`angle10`、`conf` 和 `err`。
- 推荐超时：`PING` 超过 `500ms` 无 ACK 记 `VISION_OFFLINE`；`SPOT` 超过 `150ms` 未更新记 `VISION_SPOT_STALE`；`A4_LOCKED` 等待超过 `3000~5000ms` 记 `VISION_A4_TIMEOUT`；一次性 `A4_LOCKED` 可每 `100ms` 重发，最多 `5` 次。
- 状态对应：`BOOT` 低频 `PING/STATUS`；`CALIB_SCREEN/RESET_RUN/SQUARE_RUN/A4_RUN` 使用 `SPOT640`；`A4_DETECT/A4_READY` 使用 `A4GRAY + LOCK_A4`；`PAUSED/DONE/ERROR` 由 TI 先制动，再按需发 `STOP`。

## 验证
- 题目硬指标：
  - 红色复位：任意位置回原点，中心距原点误差 `<=2cm`。
  - 红色屏幕边线：30 秒内沿 `0.5m x 0.5m` 正方形边线顺时针一周，中心距边线 `<=2cm`。
  - 红色 A4 靶纸：30 秒内沿约 `1.8cm` 黑色电工胶带边线顺时针一周；完全脱离胶带一次扣分，连续脱离 5cm 以上为 0 分。
  - 红色旋转 A4 靶纸：A4 靶纸任意旋转角度和任意位置，要求同上。
  - 绿色追踪：启动后 2 秒内追踪成功并连续声光提示，两个光斑中心距离 `<=3cm`。
  - 追踪过程：2 秒后中心距离大于 3cm 记一次失败，连续失败 3 秒以上为 0 分。
  - 暂停：红色、绿色系统均需暂停键；同时按下暂停键后两光斑立即制动，便于测距。
- 最小 build command：`powershell -NoProfile -ExecutionPolicy Bypass -File "E:\23e\MSPM0G3507-red\firmware\mspm0g3507-red\.vscode\ti-build-flash.ps1"`。
- Flash/run method：`powershell -NoProfile -ExecutionPolicy Bypass -File "E:\23e\MSPM0G3507-red\firmware\mspm0g3507-red\.vscode\ti-build-flash.ps1" -Flash`。
- MaixCAM app artifact：`modular/maixcampro/dist/maix-red_vision-v0.1.0.zip`。
- MaixCAM UART smoke checks：上板运行后确认启动标识；TI 或串口工具发送 `>T,1,PING` 应收到 `ACK` 和 `STATUS`；发送 `>T,2,MODE,SPOT640` 后应持续收到 `SPOT`，红点照射屏幕时 `valid=1` 且 `x/y/conf/lat` 更新。
- MaixCAM A4 smoke checks：发送 `>T,3,MODE,A4GRAY` 后观察 overlay 的外框、内孔、中心线、`stable/conf/angle`；未稳定时 `>T,4,LOCK_A4` 应返回 `ACK,LOCK_A4,0,4` 和 `ERR,4`，稳定后应返回 `A4_LOCKED,frame,640,480,x0,y0,x1,y1,x2,y2,x3,y3,angle10,conf,err`。
- On-board smoke checks：先验证 MaixCAM 红点坐标流，再验证舵机/电机云台两轴零点、行程、限位、暂停制动，随后验证红色复位和四边路径。

## 护栏
- 基本要求（3）、（4）未得分时，不进行发挥部分（2）测试；工程优先级应先保红色 A4 和旋转 A4。
- 绿色追踪不能依赖红色控制器提供坐标；必须通过自身传感/视觉/光斑检测闭环追踪。
- A4 靶纸位置和旋转角度任意，红色控制若无外部感知则需要明确校准流程，否则无法覆盖基本要求（4）。
- 当前 MaixCAM A4 检测为 640x480 测试版，TI 端只能在收到稳定 `A4_LOCKED` 后使用角点；A4 路径前必须上板确认黑色阈值、角点顺序、640x480 坐标尺度和最差光照稳定性。
- 当前 MaixCAM 启动默认 `VISION_SPOT640`，若 TI 端实现严格主控握手，需要决定是否保持该默认行为或改为上电 `VISION_IDLE`。
- 所有硬件参数、舵机角度到屏幕坐标映射、限位、PID/速度参数应可现场校准；是否运行时调参或持久化待确认。
