# 项目地图

## 目标
- 当前比赛目标：2023 全国大学生电子设计竞赛本科组 E 题“运动目标控制与自动追踪系统”的绿色光斑自动追踪子系统。
- 绿色系统职责：在红色目标复位或执行 A4 靶纸路径时，独立观测屏幕并控制绿色二维云台，使绿色光斑追踪红色光斑。
- 发挥部分验收边界：一键启动后 2 秒内追踪成功并连续声光提示；两光斑中心距离 `<=3cm`；2 秒后距离 `>3cm` 记一次失败，连续失败 3 秒以上为 0 分。
- 明确非目标：绿色系统不得通过任何方式接收红色系统坐标、状态、路径、时序或控制消息；屏幕上不能放电子元件；不能使用台式机或笔记本电脑作为控制系统。

## 硬件
- 绿色系统主控：TI 端，使用 `MSPM0G3507`；当前仓库尚无固件工程。
- 绿色系统视觉：`MaixCAM2`，通过 `UART2` 与 TI 端视觉串口通信。
- 执行器：绿色激光笔固定在独立二维电控云台上，需能立即制动以支持暂停测距。
- 传感/视觉：MaixCAM2 独立观测屏幕上的红色光斑和绿色光斑，输出双点坐标、像素误差和标定后的距离。
- 声光提示：TBD；追踪成功后应连续声光提示，追踪丢失或暂停时状态必须可区分。
- 按键：绿色 TI 端使用 3 个按键；`K1` 启动/确认，`K2` 暂停/恢复，`K3` 停止/复位。
- 摆放约束：绿色激光笔放置在线段上任意位置；线段与屏幕平行，距红色激光笔大于 `0.4m`、小于 `1m`。
- 视觉通信硬件：TI 侧沿用红色工程接法，使用 `UART1 PA8/PA9`；MaixCAM2 侧使用 `UART2`；`TX->RX`、`RX->TX` 交叉连接，两端共地，`3.3V TTL UART`。
- 外设归属：待工程建立后记录 PWM/TIM/GPIO/UART/ADC/I2C/SPI/camera/按键/蜂鸣器/指示灯的唯一初始化所有权；视觉通信应分为 TI 侧 `bsp_vision_uart` 和应用层 `app_green_vision_comm`。

## 代码地图
- `PROJECT.md`：当前绿色项目地图。
- `docs/state_machines.html`：绿色系统状态机大框架预览；当前固定顶层状态、子状态机分工和运行期粗流程。
- `modular/maixcam2/`：绿色 MaixCAM2 视觉侧 MaixPy 应用；当前包含 `main.py`、`vision_spot.py`、`vision_green.py`、`protocol.py`、`config.py`、`app.yaml` 和目录规则 `AGENTS.md`。
- `modular/maixcam2/main.py`：MaixCAM2 运行入口、视觉状态机、UART 命令分发、周期 `STATUS/TRACK` 输出。
- `modular/maixcam2/vision_spot.py`：红色激光点传统视觉检测；原样沿用红色工程红点检测逻辑。
- `modular/maixcam2/vision_green.py`：绿色激光点传统视觉检测；使用独立绿色 LAB 阈值、上一帧 ROI 跟踪、候选过滤和亮芯精定位。
- `modular/maixcam2/protocol.py`：绿色 TI-MaixCAM2 ASCII CSV 协议封包、解包和非阻塞串口轮询。
- `modular/maixcam2/config.py`：MaixCAM2 UART2、双光斑帧率、红绿阈值、ROI、候选过滤和亮芯精定位参数。
- `2023_E_2023 E题 运动目标控制与自动追踪系统.pdf`：题目来源。
- 待讨论分层：MaixCAM2 视觉侧、MSPM0G3507 控制侧、通信协议和外设驱动边界尚未最终确定。

## 运行路径
- TI Init order：TBD，待固件工程建立。
- TI Main loop/task order：TBD，待固件工程建立。
- MaixCAM2 Init order：`main.py` 启动后打印启动标识，复用 `B0/B1` 为 `UART2_TX/RX`，打开 `/dev/ttyS2`，初始化时间戳，并发送一帧 `STATUS`。
- MaixCAM2 Main loop order：轮询 UART 命令；`VISION_TRACK640` 读取固定 `640x480` 相机帧，同帧检测红点和绿点，计算 `err_x/err_y = red - green`，最后按周期输出 `STATUS` 或 `TRACK`。
- MaixCAM2 camera/display：当前固定使用 `camera.Camera(640, 480, fps=60)` 和 `display.Display()`；调试 overlay 显示红绿检测框、ROI、双点有效性、误差、置信度和耗时。
- 重要 callback/interrupt dispatch：TBD，待固件工程建立。
- 推荐运行形态：TBD；中断/HAL callback 只收集按键、串口或传感帧事件，不做追踪决策。

## 状态机总览
- 详表：`docs/state_machines.html`。
- 当前大框架：`preview`；顶层 owner 暂定 `app_green_system` / `GreenSystem`，只负责比赛流程、安全边界和子状态机启停。
- 按键事件入口：预定由 `GreenSystem_OnKey()` 接收 `GREEN_KEY_K1_SHORT/GREEN_KEY_K2_SHORT/GREEN_KEY_K3_SHORT`；按键 BSP 只做消抖和事件上报，不直接改状态。
- `vision_app`：MaixCAM2 侧已实现 `VISION_IDLE/VISION_TRACK640/VISION_ERROR`；`VISION_TRACK640` 输出 `TRACK` 双光斑坐标流。
- 子状态机分工：视觉有效性、追踪质量、云台输出、声光提示；后续逐个细化。
- 关键 STATUS 字段：`state`、`run_phase`、`vision_valid`、`stale_ms`、`dist_mm`、`fail_ms`、`pause`、`last_reason`、`out_x/out_y`。

## 接入规则
- 绿色追踪决策不放进 PWM driver、视觉 driver、按键中断或 HAL callback。
- 按键处理不直接控制云台、声光或视觉私有状态；只能调用 `GreenSystem_OnKey()` 或等价公开 API。
- 绿色系统只能使用自身传感/视觉得到的红色光斑位置，不得读取红色控制器内部变量、串口消息、共享总线或同步信号。
- 追踪成功、失败累计和 2 秒/3 秒计时必须有唯一 owner，避免多个 loose flags 同时改状态。
- 暂停键必须优先于普通追踪输出；进入暂停时立即停止云台运动，保留当前光斑位置便于测距。
- 声光提示 owner 待定；锁定、失败、暂停和故障状态必须可区分。

## TI-MaixCAM2 通信协议
- 物理层：`3.3V TTL UART`，共地，默认 `115200 8N1`；TI 侧使用 `UART1 PA8/PA9`，MaixCAM2 侧使用 `UART2`。
- 连接方式：`TX->RX`、`RX->TX` 交叉连接；连接方式与红色工程一致，但红绿系统之间不得通信。
- 包格式：ASCII CSV，一行一包，`\r\n` 结尾；`#` 开头为视觉调试行，TI 忽略。
- 方向：TI 是命令方和追踪状态机 owner；MaixCAM2 是视觉检测 owner，只输出观测结果、自身状态和错误原因。
- 命令格式：`>T,<seq>,<cmd>[,<arg>...]`；回包格式：`<C,<seq>,<type>[,<field>...]`；`seq` 由 TI 递增。
- TI 命令：`PING`、`MODE,TRACK640`、`STOP`、`STATUS`。
- MaixCAM2 回包：`ACK,cmd,ok,err`、`STATUS,mode,fps10,frame,err`、`TRACK,frame,iw,ih,track_valid,red_valid,green_valid,dist_valid,red_x,red_y,green_x,green_y,err_x,err_y,dist_mm,red_conf,green_conf,lat,err`、`ERR,code`。
- `TRACK640`：MaixCAM2 固定输出 `640x480` 双光斑观测；`err_x/err_y = red - green`，单位为像素；`dist_mm` 为标定后的红绿中心距离，未标定或不可信时 `dist_valid=0` 且 `dist_mm=-1`。
- 时序约定：`TRACK` 目标周期 `20ms`，约 `50Hz`；`STATUS` 低频输出周期 `1500ms`；TI 侧 `TRACK` 超过 `150ms` 未更新即判定为 stale。
- 推荐超时：`PING` 超过 `500ms` 无 ACK 记 `VISION_OFFLINE`；`TRACK` 超过 `150ms` 未更新记 `VISION_TRACK_STALE`；通信超过 `1000ms` 无任何回包记 `VISION_OFFLINE`。
- 按键联动：`K1` 启动时 TI 发送 `MODE,TRACK640`；`K2` 暂停/恢复只影响 TI 侧云台制动，MaixCAM2 继续输出 `TRACK`；`K3` 停止/复位时 TI 发送 `STOP`。

## 按键事件契约
- `K1` 短按：启动/确认。`IDLE` 接收后进入 `PREPARE`；准备成功后发送 `MODE,TRACK640` 并进入 `RUN_ACTIVE`。非 `IDLE` 状态下默认不重复启动。
- `K2` 短按：暂停/恢复。`RUN_ACTIVE` 接收后进入 `PAUSED` 并立即制动云台；`PAUSED` 接收后回到 `RUN_ACTIVE`。MaixCAM2 不停止，继续输出 `TRACK` 供测距和恢复。
- `K3` 短按：停止/复位。任意状态接收后进入 `IDLE`，云台安全输出，关闭普通追踪声光，清失败累计，并向 MaixCAM2 发送 `STOP`。
- 长按/组合键：当前不定义，避免早期状态机入口过多；后续如需标定或调参再单独扩展。

## 验证
- 最小 build command：TBD，待 MSPM0G3507 工程建立。
- Flash/run method：TBD。
- PC-side checks：MaixCAM2 视觉侧可用 `python -m py_compile modular\maixcam2\config.py modular\maixcam2\protocol.py modular\maixcam2\vision_spot.py modular\maixcam2\vision_green.py modular\maixcam2\main.py` 做语法检查。
- On-board smoke checks：确认按键事件、云台立即制动、绿点可被自身传感链路观测、红点不通信也能被观测、声光提示随锁定/失锁/暂停变化。
- Key smoke checks：先用 mock 或串口命令模拟 `K1/K2/K3`，确认 `IDLE/PREPARE/RUN_ACTIVE/PAUSED` 跳转、`MODE,TRACK640/STOP` 下发和云台制动标志；再接真实按键 BSP。
- MaixCAM2 UART smoke checks：TI 或串口工具发送 `>T,1,PING` 应收到 `ACK` 和 `STATUS`；发送 `>T,2,MODE,TRACK640` 后应约每 `20ms` 收到一帧 `TRACK`，红点和绿点照射屏幕时 `track_valid=1` 且坐标、误差、置信度和延迟更新。
- 题目硬指标：启动后 2 秒内 `<=3cm`；运行中 `>3cm` 记失败；连续失败 3 秒以上为 0 分；暂停时红绿两系统均立即制动。

## 护栏
- 基本要求（3）、（4）未得分时，不进行发挥部分（2）测试；绿色系统应优先支持发挥部分（1）和可重复追踪验证。
- 绿色追踪不能依赖红色路径规划或红色系统通信；只能通过自身观测闭环追踪。
- 未完成标定前，不把像素误差硬编码成毫米距离；`dist_mm` 需要来自屏幕尺度/A4 尺度/标定结果。
- 不要为同一外设重复创建初始化代码；工程建立后必须记录唯一外设所有者。
- 没有源码时，`docs/state_machines.html` 只作为讨论框架，不代表已授权或已实现固件行为。
