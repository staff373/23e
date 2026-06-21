from maix import image  # 导入 MaixPy 图像模块用于颜色常量和画图。
import config  # 导入现场可调参数配置模块。

g_last_x = -1  # 保存上一帧有效绿点 X 坐标。
g_last_y = -1  # 保存上一帧有效绿点 Y 坐标。
g_lost_count = config.LOST_TO_FULL  # 保存连续丢失帧数，初始强制全屏搜索。
g_last_roi = None  # 保存当前用于调试显示的 ROI。
g_last_full = True  # 保存当前是否使用全屏搜索。

def reset():  # 重置绿点跟踪器状态。
    global g_last_x, g_last_y, g_lost_count, g_last_roi, g_last_full  # 声明要修改跟踪器全局状态。
    g_last_x = -1  # 清空上一帧 X 坐标。
    g_last_y = -1  # 清空上一帧 Y 坐标。
    g_lost_count = config.LOST_TO_FULL  # 设置为丢失状态以触发全屏搜索。
    g_last_roi = None  # 清空上一帧 ROI。
    g_last_full = True  # 标记下一帧使用全屏搜索。

def clamp(value, low, high):  # 把数值限制在指定范围内。
    if value < low:  # 判断数值是否低于下限。
        return low  # 返回下限。
    if value > high:  # 判断数值是否高于上限。
        return high  # 返回上限。
    return value  # 返回原始数值。

def cfg(name, default):  # 读取可调参数，兼容 IDE 漏同步旧 config.py 的情况。
    return getattr(config, name, default)  # 返回 config 中的值，缺失时使用本文件默认值。

def current_roi():  # 计算当前帧应该使用的搜索区域。
    global g_last_roi, g_last_full  # 声明要修改调试用 ROI 状态。
    if g_lost_count >= config.LOST_TO_FULL or g_last_x < 0 or g_last_y < 0:  # 判断是否需要全屏搜索。
        g_last_roi = None  # 清空 ROI 表示全屏。
        g_last_full = True  # 标记当前为全屏搜索。
        return None  # 返回空 ROI 让 find_blobs 扫描整幅图。
    half = config.ROI_SIZE // 2  # 计算 ROI 半边长。
    x = clamp(g_last_x - half, 0, config.TRACK_IMG_W - config.ROI_SIZE)  # 计算并限制 ROI 左上角 X。
    y = clamp(g_last_y - half, 0, config.TRACK_IMG_H - config.ROI_SIZE)  # 计算并限制 ROI 左上角 Y。
    g_last_roi = [x, y, config.ROI_SIZE, config.ROI_SIZE]  # 保存当前 ROI。
    g_last_full = False  # 标记当前为 ROI 搜索。
    return g_last_roi  # 返回 ROI 给 find_blobs。

def blob_rect(blob):  # 提取 MaixPy blob 的矩形字段。
    return int(blob[0]), int(blob[1]), int(blob[2]), int(blob[3])  # 返回 x、y、w、h。

def blob_center(blob):  # 计算 blob 中心点坐标。
    return int(blob[5]), int(blob[6])  # 返回 MaixPy 色块质心作为候选中心点。

def make_blob_like(x, y, w, h, cx, cy, pixels=0):  # 生成兼容现有 blob 访问方式的虚拟候选。
    return [int(x), int(y), int(w), int(h), int(pixels), int(cx), int(cy)]  # 返回包含矩形、像素数和质心字段的列表。

def blob_pixels(blob):  # 提取 MaixPy blob 的阈值命中像素数。
    return int(blob[4])  # 返回 find_blobs 提供的 pixels 字段，虚拟候选也保持同一索引。

def blob_density(blob):  # 计算候选外框内的绿色像素填充率百分比。
    x, y, w, h = blob_rect(blob)  # 读取候选矩形以计算外框面积。
    area = max(w * h, 1)  # 计算外框面积并避免除零。
    return blob_pixels(blob) * 100 // area  # 返回绿色阈值像素占外框面积的百分比。

def blob_core_roi(blob):  # 计算白芯精定位使用的小搜索框。
    x, y, w, h = blob_rect(blob)  # 读取候选绿色外环矩形。
    pad = config.CORE_ROI_PAD  # 读取白芯搜索框外扩边距。
    cx, cy = blob_center(blob)  # 读取候选质心作为搜索框中心。
    max_side = cfg("CORE_ROI_MAX_SIDE", 36)  # 读取亮芯搜索框最大边长，兼容旧配置缺失。
    x0 = clamp(x - pad, 0, config.TRACK_IMG_W - 1)  # 从绿色候选外框左侧外扩，避免只围绕偏移质心搜索。
    y0 = clamp(y - pad, 0, config.TRACK_IMG_H - 1)  # 从绿色候选外框上侧外扩，覆盖被黑框切裂的光斑。
    x1 = clamp(x + w + pad, x0 + 1, config.TRACK_IMG_W)  # 从绿色候选外框右侧外扩，并保证搜索框至少一列。
    y1 = clamp(y + h + pad, y0 + 1, config.TRACK_IMG_H)  # 从绿色候选外框下侧外扩，并保证搜索框至少一行。
    if x1 - x0 > max_side:  # 判断搜索框宽度是否超过现场性能上限。
        x0 = clamp(cx - max_side // 2, 0, config.TRACK_IMG_W - max_side)  # 围绕候选质心裁剪过宽搜索框。
        x1 = x0 + max_side  # 保持裁剪后的搜索框宽度固定。
    if y1 - y0 > max_side:  # 判断搜索框高度是否超过现场性能上限。
        y0 = clamp(cy - max_side // 2, 0, config.TRACK_IMG_H - max_side)  # 围绕候选质心裁剪过高搜索框。
        y1 = y0 + max_side  # 保持裁剪后的搜索框高度固定。
    return [x0, y0, x1 - x0, y1 - y0]  # 返回 MaixPy 矩形格式的白芯搜索框。

def pixel_rgb(img, x, y):  # 读取指定像素的 RGB 三通道。
    pixel = img.get_pixel(x, y, True)  # 使用 MaixPy 官方 get_pixel 读取拆分后的 RGB 值。
    if isinstance(pixel, int):  # 兼容少数固件返回 RGB565 整数的情况。
        r = ((pixel >> 11) & 31) * 255 // 31  # 从 RGB565 中还原红通道。
        g = ((pixel >> 5) & 63) * 255 // 63  # 从 RGB565 中还原绿通道。
        b = (pixel & 31) * 255 // 31  # 从 RGB565 中还原蓝通道。
        return r, g, b  # 返回还原后的 RGB 三通道。
    return int(pixel[0]), int(pixel[1]), int(pixel[2])  # 返回列表或元组形式的 RGB 三通道。

def core_pixel_ok(r, g, b):  # 判断像素是否像绿色激光的泛白高亮中心。
    if r < config.GREEN_CORE_MIN_R or g < config.GREEN_CORE_MIN_G or b < config.GREEN_CORE_MIN_B:  # 判断单通道亮度是否不足。
        return False  # 亮度不足时不是白芯。
    if r + g + b < config.GREEN_CORE_MIN_BRIGHT:  # 判断总亮度是否不足。
        return False  # 总亮度不足时不是白芯。
    return True  # 亮度足够即可参与白芯中心计算，绿色身份由外层绿光晕候选提供。

def core_pixel_score(r, g, b):  # 计算亮芯峰值像素评分。
    green_gain = max(green_advantage(r, g, b), 0) * cfg("GREEN_CORE_SCORE_GREEN_GAIN", 2)  # 把绿色优势作为附加权重，压制普通白反光。
    return r + g + b + green_gain  # 返回总亮度叠加绿色优势后的峰值评分。

def core_cluster_ok(x, y, r, g, b, peak_x, peak_y, peak_bright):  # 判断像素是否属于最亮小簇。
    dx = x - peak_x  # 计算像素相对峰值的 X 距离。
    dy = y - peak_y  # 计算像素相对峰值的 Y 距离。
    radius = cfg("CORE_CLUSTER_RADIUS", 4)  # 读取亮芯小簇半径。
    if dx * dx + dy * dy > radius * radius:  # 判断像素是否离峰值太远。
        return False  # 离峰值太远时不参与中心计算。
    if r + g + b < peak_bright - cfg("CORE_PEAK_MARGIN", 28):  # 判断像素是否明显暗于最亮峰值。
        return False  # 明显暗于峰值时不参与中心计算。
    return core_pixel_ok(r, g, b)  # 复用亮芯基础约束，避免普通背景进入小簇。

def refine_core_center(img, blob):  # 在绿色候选附近用白芯像素精定位中心。
    roi = blob_core_roi(blob)  # 计算白芯搜索区域。
    peak_x = -1  # 保存亮芯峰值 X 坐标。
    peak_y = -1  # 保存亮芯峰值 Y 坐标。
    peak_score = -1  # 保存亮芯峰值评分。
    peak_bright = 0  # 保存亮芯峰值 RGB 总亮度。
    for y in range(roi[1], roi[1] + roi[3]):  # 第一遍扫描搜索框内的每一行以寻找局部峰值。
        for x in range(roi[0], roi[0] + roi[2]):  # 第一遍扫描搜索框内的每一列以寻找局部峰值。
            r, g, b = pixel_rgb(img, x, y)  # 读取当前像素 RGB。
            if core_pixel_ok(r, g, b):  # 判断当前像素是否满足亮芯基础条件。
                score = core_pixel_score(r, g, b)  # 计算当前像素的亮芯峰值评分。
                if score > peak_score:  # 判断当前像素是否优于已知峰值。
                    peak_x = x  # 记录新的峰值 X 坐标。
                    peak_y = y  # 记录新的峰值 Y 坐标。
                    peak_score = score  # 记录新的峰值评分。
                    peak_bright = r + g + b  # 记录新的峰值总亮度。
    if peak_score < 0:  # 判断是否没有找到可靠亮芯峰值。
        cx, cy = blob_center(blob)  # 回退到绿色候选质心。
        return cx, cy, 0, roi  # 返回回退中心和白芯命中数。
    sum_x = 0  # 累计白芯像素的加权 X 坐标。
    sum_y = 0  # 累计白芯像素的加权 Y 坐标。
    sum_w = 0  # 累计白芯像素亮度权重。
    count = 0  # 统计命中的白芯像素数量。
    for y in range(roi[1], roi[1] + roi[3]):  # 第二遍扫描搜索框内的每一行以聚合最亮小簇。
        for x in range(roi[0], roi[0] + roi[2]):  # 第二遍扫描搜索框内的每一列以聚合最亮小簇。
            r, g, b = pixel_rgb(img, x, y)  # 读取当前像素 RGB。
            if core_cluster_ok(x, y, r, g, b, peak_x, peak_y, peak_bright):  # 判断当前像素是否属于峰值附近亮芯小簇。
                weight = core_pixel_score(r, g, b)  # 使用亮度叠加绿色优势作为中心权重。
                sum_x = sum_x + x * weight  # 累计加权 X 坐标。
                sum_y = sum_y + y * weight  # 累计加权 Y 坐标。
                sum_w = sum_w + weight  # 累计权重。
                count = count + 1  # 累计白芯像素数量。
    if count <= 0 or sum_w <= 0:  # 判断是否没有找到可靠白芯。
        return peak_x, peak_y, 1, roi  # 回退到单个峰值像素，避免红晕质心把锁点拉偏。
    return int(sum_x / sum_w), int(sum_y / sum_w), count, roi  # 返回亮度加权白芯中心和命中数。

def green_advantage(r, g, b):  # 计算绿色相对红蓝通道的优势。
    return g - max(r, b)  # 返回 g - max(r, b)，纯白高光通常接近 0。

def green_adv_score(delta):  # 把绿色优势换算为候选加分。
    green_adv_min = cfg("GREEN_ADV_MIN", 10)  # 读取最低绿色优势。
    if delta < green_adv_min:  # 判断是否低于最低绿色优势。
        return 0  # 绿色优势不足时不给分。
    green_score_max = cfg("GREEN_ADV_SCORE_MAX", 35)  # 读取绿色优势最高加分。
    span = cfg("GREEN_ADV_STRONG", 55) - green_adv_min  # 计算从最低值到强绿值的跨度。
    if span <= 0:  # 防止配置非法导致除零。
        return green_score_max  # 配置非法时直接给满分，避免运行异常。
    score = (delta - green_adv_min) * green_score_max // span  # 线性映射到配置分值。
    return clamp(score, 1, green_score_max)  # 限制评分范围。

def candidate_green_adv(img, blob):  # 用少数采样点估计候选的绿色优势。
    cx, cy = blob_center(blob)  # 读取候选质心。
    r = cfg("GREEN_PROBE_RADIUS", 2)  # 读取十字采样半径。
    x0, y0, w, h = blob_rect(blob)  # 读取候选外框，用网格覆盖大光晕边缘。
    grid = max(cfg("GREEN_ADV_SAMPLE_GRID", 3), 1)  # 读取绿色优势网格数量并保证至少一格。
    best = -255  # 初始化最大绿色优势。
    samples = ((0, 0), (-r, 0), (r, 0), (0, -r), (0, r))  # 只采样中心十字，保持常数级开销。
    for dx, dy in samples:  # 遍历少数采样点。
        x = clamp(cx + dx, 0, config.TRACK_IMG_W - 1)  # 限制 X 采样坐标。
        y = clamp(cy + dy, 0, config.TRACK_IMG_H - 1)  # 限制 Y 采样坐标。
        rr, gg, bb = pixel_rgb(img, x, y)  # 读取采样点 RGB。
        delta = green_advantage(rr, gg, bb)  # 计算绿色优势。
        if delta > best:  # 判断是否找到更强绿色优势。
            best = delta  # 保存最大绿色优势。
    for gy in range(grid):  # 遍历候选外框纵向网格采样点。
        y = clamp(y0 + (gy + 1) * h // (grid + 1), 0, config.TRACK_IMG_H - 1)  # 计算网格采样 Y 坐标。
        for gx in range(grid):  # 遍历候选外框横向网格采样点。
            x = clamp(x0 + (gx + 1) * w // (grid + 1), 0, config.TRACK_IMG_W - 1)  # 计算网格采样 X 坐标。
            rr, gg, bb = pixel_rgb(img, x, y)  # 读取候选框内采样点 RGB。
            delta = green_advantage(rr, gg, bb)  # 计算候选框内采样点绿色优势。
            if delta > best:  # 判断网格采样是否找到更强绿色优势。
                best = delta  # 保存最大绿色优势。
    return best  # 返回候选附近最大绿色优势。

def current_green_min_adv():  # 根据当前搜索状态选择绿色优势下限。
    if g_last_full:  # 判断当前是否为全屏重新捕获。
        return cfg("GREEN_FULL_MIN_ADV", 8)  # 全屏捕获时使用更高下限，降低白色反光首次抢锁概率。
    return cfg("GREEN_LOCK_MIN_ADV", 3)  # 已锁定 ROI 内使用较低下限，保留黑框内弱绿点。

def core_green_support(img, cx, cy):  # 围绕最终亮芯位置采样绿色光晕支撑。
    base = max(cfg("GREEN_PROBE_RADIUS", 2), 1)  # 读取基础采样半径并保证至少一像素。
    step = max(base * cfg("GREEN_ADV_SAMPLE_GRID", 3), 3)  # 复用已有采样步长参数，不扩大亮芯搜索窗口。
    near = step  # 设置近环采样距离，用于贴近亮芯寻找绿光晕。
    far = min(step * 2, cfg("GREEN_MERGE_BLOB_DIST", 24))  # 设置远环采样距离，兼容较大的白色饱和中心。
    min_adv = current_green_min_adv()  # 读取当前模式下绿色优势下限。
    best = -255  # 初始化周围采样最大绿色优势。
    count = 0  # 初始化满足绿色支撑的方向数量。
    samples = ((-near, 0), (near, 0), (0, -near), (0, near), (-far, 0), (far, 0), (0, -far), (0, far))  # 采样水平和垂直两圈，避免依赖黑色背景判断。
    for dx, dy in samples:  # 遍历亮芯周围少量固定采样点。
        x = clamp(cx + dx, 0, config.TRACK_IMG_W - 1)  # 限制 X 采样坐标。
        y = clamp(cy + dy, 0, config.TRACK_IMG_H - 1)  # 限制 Y 采样坐标。
        r, g, b = pixel_rgb(img, x, y)  # 读取采样点 RGB。
        adv = green_advantage(r, g, b)  # 计算采样点绿色优势。
        if adv > best:  # 判断是否刷新最大绿色优势。
            best = adv  # 保存最大绿色优势。
        if adv >= min_adv:  # 判断该方向是否存在有效绿色光晕支撑。
            count = count + 1  # 累计有效绿色支撑方向。
    return count, best  # 返回支撑方向数量和最大绿色优势。

def core_support_ok(core_count, support_count, green_adv):  # 判断最终亮芯是否有足够绿色身份支撑。
    min_adv = current_green_min_adv()  # 读取当前搜索模式允许的最低绿色优势。
    if core_count > 0 and support_count > 0:  # 有亮芯且周围有绿光晕时可信度最高。
        return True  # 接受该候选。
    if core_count > 0 and green_adv >= min_adv:  # 亮芯在黑框内可能只有弱绿色候选支撑，达到当前模式下限即可保留。
        return True  # 接受该候选。
    if core_count <= 0 and support_count > 0 and green_adv >= cfg("GREEN_JUMP_ACCEPT_ADV", 22):  # 无亮芯时只允许强绿色光晕临时兜底。
        return True  # 接受该候选。
    return False  # 绿色身份不足时拒绝候选，避免锁到黑框边缘光晕。

def ratio10(w, h):  # 计算候选框宽高比乘以十。
    if h <= 0 or w <= 0:  # 判断候选尺寸是否非法。
        return 999  # 返回一个极大比值表示非法候选。
    if w > h:  # 判断宽是否大于高。
        return int(w * 10 / h)  # 返回宽高比乘以十。
    return int(h * 10 / w)  # 返回高宽比乘以十。

def candidate_ok(blob):  # 判断候选 blob 是否像绿色激光点。
    x, y, w, h = blob_rect(blob)  # 读取候选矩形。
    area = w * h  # 计算候选框面积。
    max_area = config.MAX_AREA * 6  # 放宽绿色光晕面积上限，避免大光晕在白芯定位前被丢弃。
    max_side = config.MAX_SIDE * 3  # 放宽绿色光晕边长上限，允许白纸外的大团光晕进入白芯搜索。
    if area < config.MIN_AREA or area > max_area:  # 判断候选面积是否在合理范围外。
        return False  # 面积不合理则丢弃。
    if w > max_side or h > max_side:  # 判断候选边长是否过大。
        return False  # 边长过大则丢弃。
    if blob_density(blob) < cfg("GREEN_MIN_DENSITY", 8):  # 判断候选绿色像素是否过于稀疏。
        return False  # 过稀疏的边缘反光块不参与后续亮芯定位。
    if ratio10(w, h) > config.MAX_RATIO10:  # 判断候选是否过扁。
        return False  # 形状过扁则丢弃。
    return True  # 候选通过基础过滤。

def candidate_pre_score(blob):  # 用不读像素的轻量分数给绿色候选排序。
    x, y, w, h = blob_rect(blob)  # 读取候选矩形用于面积和形状评分。
    cx, cy = blob_center(blob)  # 读取候选中心用于连续性评分。
    area = w * h  # 计算候选框面积。
    compact = 30 - min(ratio10(w, h), 30)  # 计算候选形状紧凑度，越接近圆形越高。
    size = min(area, 120) * 25 // 120  # 计算候选尺寸基础分，避免背景大块完全压过小光斑。
    continuity = distance_score(cx, cy)  # 复用上一帧连续性，优先保留锁定点附近候选。
    return compact + size + continuity  # 返回不依赖 get_pixel 的预筛分数。

def trim_candidates(candidates):  # 限制进入绿点分组和像素采样的候选数量。
    limit = 16  # 固定保留最多 16 个绿色候选，防止复杂画面碎块数量拖垮单帧处理。
    if len(candidates) <= limit:  # 判断候选数量是否已经在安全范围内。
        return candidates  # 数量不多时保持原列表，避免额外排序开销。
    ranked = []  # 保存预筛分数和候选引用。
    for blob in candidates:  # 遍历所有基础过滤后的候选。
        ranked.append((candidate_pre_score(blob), -len(ranked), blob))  # 记录分数和唯一序号，避免同分时比较 blob 对象。
    ranked.sort(reverse=True)  # 按预筛分数从高到低排序，优先保留小而连续的候选。
    kept = []  # 保存截断后的候选列表。
    for i in range(limit):  # 只取固定数量的最高分候选。
        kept.append(ranked[i][2])  # 取回原始 blob 供后续合并和精定位使用。
    return kept  # 返回数量受控的候选列表。

def locked_tracking():  # 判断当前是否处于上一帧有效锁定状态。
    return g_last_x >= 0 and g_last_y >= 0 and g_lost_count < config.LOST_TO_FULL  # 返回是否可使用上一帧位置。

def distance2_from_last(cx, cy):  # 计算候选与上一帧位置的平方距离。
    dx = cx - g_last_x  # 计算 X 方向位移。
    dy = cy - g_last_y  # 计算 Y 方向位移。
    return dx * dx + dy * dy  # 返回平方距离。

def jump_from_last(cx, cy):  # 判断候选是否相对上一帧发生突跳。
    if not locked_tracking():  # 没有上一帧锁定状态时不判突跳。
        return False  # 返回非突跳。
    lock_max_step = cfg("LOCK_MAX_STEP", 8)  # 读取最大允许单帧位移。
    return distance2_from_last(cx, cy) > lock_max_step * lock_max_step  # 超过配置位移则判为突跳。

def distance_score(cx, cy):  # 计算候选与上一帧位置的连续性得分。
    if g_last_x < 0 or g_last_y < 0 or g_lost_count >= config.LOST_TO_FULL:  # 判断是否没有可用上一帧。
        return 20  # 没有上一帧时给中性连续性分。
    dist2 = distance2_from_last(cx, cy)  # 计算平方距离避免开方。
    lock_max_step = cfg("LOCK_MAX_STEP", 8)  # 读取最大允许单帧位移。
    if dist2 <= lock_max_step * lock_max_step:  # 判断是否在正常单帧位移内。
        return 35  # 非常连续给高分。
    if dist2 <= 400:  # 判断是否在 20 像素内。
        return 15  # 小幅偏离给中分。
    return -20  # 突跳候选主动扣分，避免黑框边缘抢锁。

def candidate_score(img, blob):  # 给候选绿点打分。
    x, y, w, h = blob_rect(blob)  # 读取候选矩形。
    cx, cy = blob_center(blob)  # 计算候选中心。
    green_adv = candidate_green_adv(img, blob)  # 计算候选绿色优势。
    green_score = green_adv_score(green_adv)  # 把绿色优势换算为评分。
    area = w * h  # 计算候选面积。
    compact = 30 - min(ratio10(w, h), 30)  # 计算形状紧凑得分。
    size = min(area, 120) * 25 // 120  # 计算候选尺寸得分。
    continuity = distance_score(cx, cy)  # 计算连续性得分。
    return compact + size + continuity + green_score, green_adv  # 返回综合分和绿色优势。

def blob_distance2(blob_a, blob_b):  # 计算两个候选中心之间的平方距离。
    ax, ay = blob_center(blob_a)  # 读取第一个候选中心。
    bx, by = blob_center(blob_b)  # 读取第二个候选中心。
    dx = ax - bx  # 计算 X 方向距离。
    dy = ay - by  # 计算 Y 方向距离。
    return dx * dx + dy * dy  # 返回平方距离避免开方。

def near_blob(blob_a, blob_b):  # 判断两个绿色碎块是否属于同一个激光点候选组。
    merge_dist = cfg("GREEN_MERGE_BLOB_DIST", cfg("MERGE_BLOB_DIST", 26))  # 读取绿点专用碎块合并距离。
    return blob_distance2(blob_a, blob_b) <= merge_dist * merge_dist  # 距离足够近则认为需要合并。

def collect_blob_group(seed, candidates):  # 从一个种子候选开始收集近邻绿色碎块。
    group = [seed]  # 初始化候选组并包含种子。
    changed = True  # 标记本轮是否继续扩展到新碎块。
    while changed:  # 迭代扩展传递近邻，处理左中右多碎块。
        changed = False  # 每轮开始先假设没有新增。
        for blob in candidates:  # 遍历所有候选碎块。
            if blob in group:  # 判断该碎块是否已经在组内。
                continue  # 已在组内则跳过。
            for member in group:  # 遍历当前组内碎块。
                if near_blob(blob, member):  # 判断该碎块是否贴近组内任一碎块。
                    group.append(blob)  # 把近邻碎块加入当前组。
                    changed = True  # 标记本轮有新增以继续扩展。
                    break  # 当前碎块已加入，结束组内遍历。
    return group  # 返回收集到的绿点碎块组。

def merge_blob_group(group):  # 把同一激光点的多个绿色碎块合并为一个虚拟候选。
    x0 = config.TRACK_IMG_W - 1  # 初始化合并框左边界为最大值。
    y0 = config.TRACK_IMG_H - 1  # 初始化合并框上边界为最大值。
    x1 = 0  # 初始化合并框右边界为最小值。
    y1 = 0  # 初始化合并框下边界为最小值。
    sum_x = 0  # 累计按面积加权的中心 X。
    sum_y = 0  # 累计按面积加权的中心 Y。
    sum_w = 0  # 累计候选面积权重。
    pixels = 0  # 累计同组候选的绿色阈值像素数量。
    for blob in group:  # 遍历同组碎块。
        x, y, w, h = blob_rect(blob)  # 读取碎块矩形。
        cx, cy = blob_center(blob)  # 读取碎块中心。
        area = max(w * h, 1)  # 计算面积权重并保证非零。
        x0 = min(x0, x)  # 更新合并框左边界。
        y0 = min(y0, y)  # 更新合并框上边界。
        x1 = max(x1, x + w)  # 更新合并框右边界。
        y1 = max(y1, y + h)  # 更新合并框下边界。
        sum_x = sum_x + cx * area  # 累计面积加权 X。
        sum_y = sum_y + cy * area  # 累计面积加权 Y。
        sum_w = sum_w + area  # 累计面积权重。
        pixels = pixels + blob_pixels(blob)  # 累计绿色阈值像素数量用于合并候选诊断。
    w = max(x1 - x0, 1)  # 计算合并框宽度并保证合法。
    h = max(y1 - y0, 1)  # 计算合并框高度并保证合法。
    cx = int(sum_x / sum_w) if sum_w > 0 else x0 + w // 2  # 计算合并候选中心 X。
    cy = int(sum_y / sum_w) if sum_w > 0 else y0 + h // 2  # 计算合并候选中心 Y。
    return make_blob_like(x0, y0, w, h, cx, cy, pixels)  # 返回虚拟候选供后续白芯精定位复用。

def best_blob(img, blobs):  # 从候选列表中选出最可信绿点。
    best = None  # 初始化最佳候选为空。
    best_score = -1  # 初始化最佳得分为非法值。
    best_green_adv = 0  # 初始化最佳候选绿色优势。
    candidates = []  # 保存通过基础过滤的绿色候选碎块。
    for blob in blobs:  # 遍历所有候选色块。
        if candidate_ok(blob):  # 判断候选是否通过基础过滤。
            candidates.append(blob)  # 保存可参与合并和评分的候选碎块。
    candidates = trim_candidates(candidates)  # 截断候选数量，避免复杂画面碎块导致分组耗时爆炸。
    for blob in candidates:  # 遍历每个候选碎块作为合并种子。
        group = collect_blob_group(blob, candidates)  # 收集该碎块附近同属一个激光点的候选组。
        merged = merge_blob_group(group)  # 把近邻碎块组合并为虚拟候选。
        score, green_adv = candidate_score(img, merged)  # 按合并候选计算综合得分。
        if green_adv < current_green_min_adv():  # 判断合并候选是否缺少当前模式所需绿色优势。
            continue  # 绿色优势不足时不参与最佳候选竞争。
        score = score + min(len(group) - 1, 3) * cfg("GREEN_GROUP_BONUS", 8)  # 多碎块合并时适当加分，避免单个反光碎块抢锁。
        if score > best_score:  # 判断是否优于当前最佳候选组。
            best = merged  # 保存新的最佳合并候选。
            best_score = score  # 保存新的最佳得分。
            best_green_adv = green_adv  # 保存最佳候选绿色优势。
    return best, best_score, best_green_adv  # 返回最佳候选、得分和绿色优势。

def local_green_recapture(img):  # 在突跳时围绕上一帧位置快速重捕获黑框内绿点。
    if not locked_tracking():  # 判断是否缺少上一帧位置。
        return None  # 没有上一帧时无法本地重捕获。
    radius = cfg("LOCAL_RECAPTURE_RADIUS", 6)  # 读取本地搜索半径。
    sum_x = 0  # 累计绿色优势加权 X。
    sum_y = 0  # 累计绿色优势加权 Y。
    sum_w = 0  # 累计绿色优势权重。
    count = 0  # 统计命中像素数。
    best_adv = -255  # 保存最大绿色优势。
    x0 = clamp(g_last_x - radius, 0, config.TRACK_IMG_W - 1)  # 计算搜索左边界。
    x1 = clamp(g_last_x + radius, 0, config.TRACK_IMG_W - 1)  # 计算搜索右边界。
    y0 = clamp(g_last_y - radius, 0, config.TRACK_IMG_H - 1)  # 计算搜索上边界。
    y1 = clamp(g_last_y + radius, 0, config.TRACK_IMG_H - 1)  # 计算搜索下边界。
    for y in range(y0, y1 + 1):  # 遍历小窗口行。
        for x in range(x0, x1 + 1):  # 遍历小窗口列。
            r, g, b = pixel_rgb(img, x, y)  # 读取像素 RGB。
            adv = green_advantage(r, g, b)  # 计算绿色优势。
            if adv > best_adv:  # 判断是否刷新最大绿色优势。
                best_adv = adv  # 保存最大绿色优势。
            if adv >= cfg("LOCAL_RECAPTURE_MIN_ADV", 12):  # 判断是否像绿色激光弱点。
                sum_x = sum_x + x * adv  # 累计加权 X。
                sum_y = sum_y + y * adv  # 累计加权 Y。
                sum_w = sum_w + adv  # 累计权重。
                count = count + 1  # 累计像素数。
    if count < cfg("LOCAL_RECAPTURE_MIN_PIXELS", 2) or sum_w <= 0:  # 判断本地重捕获是否不可靠。
        return None  # 返回失败。
    return int(sum_x / sum_w), int(sum_y / sum_w), count, best_adv  # 返回本地重捕获中心。

def invalid_result(err_code=0, reject="none", adv=0, score=0):  # 生成无效绿点结果并记录拒绝原因。
    return {"valid": 0, "x": -1, "y": -1, "conf": 0, "err": err_code, "roi": g_last_roi, "core_roi": None, "core": 0, "full": g_last_full, "lost": g_lost_count, "blob": None, "adv": adv, "score": score, "reject": reject}  # 返回无效结果字典。

def hold_last_result():  # 生成短暂丢失时保持上一帧坐标的结果。
    return {"valid": 1, "x": g_last_x, "y": g_last_y, "conf": 20, "err": 0, "roi": g_last_roi, "core_roi": None, "core": 0, "full": g_last_full, "lost": g_lost_count, "blob": None, "adv": 0, "score": 20, "reject": "hold"}  # 返回低置信度保持结果。

def detect(img):  # 在一帧 RGB 图像中检测绿色激光点。
    global g_last_x, g_last_y, g_lost_count  # 声明要修改跟踪位置和丢失计数。
    roi = current_roi()  # 获取本帧搜索区域。
    if roi:  # 判断是否使用 ROI 搜索。
        blobs = img.find_blobs(config.GREEN_THRESHOLDS, roi=roi, pixels_threshold=config.MIN_PIXELS, area_threshold=config.MIN_AREA)  # 在 ROI 内寻找绿色色块。
    else:  # 处理全屏搜索。
        blobs = img.find_blobs(config.GREEN_THRESHOLDS, pixels_threshold=config.MIN_PIXELS, area_threshold=config.MIN_AREA)  # 在全屏寻找绿色色块。
    blob, score, green_adv = best_blob(img, blobs)  # 从候选中选出最佳绿点。
    if not blob:  # 判断是否没有找到可信绿点。
        g_lost_count = g_lost_count + 1  # 增加连续丢失计数。
        if g_last_x >= 0 and g_last_y >= 0 and g_lost_count <= cfg("HOLD_LAST_MAX_LOST", 1):  # 判断是否允许短暂保持上一帧坐标。
            return hold_last_result()  # 输出上一帧坐标，避免单帧阈值抖动打断控制。
        return invalid_result(0, "no_blob", green_adv, score)  # 返回无效并保留绿色证据调试信息。
    cx, cy, core_count, core_roi = refine_core_center(img, blob)  # 优先用白芯亮度加权中心精定位绿点。
    support_count, support_adv = core_green_support(img, cx, cy)  # 围绕最终中心检查绿色光晕支撑。
    if support_adv > green_adv:  # 判断亮芯周围是否比候选质心有更强绿色证据。
        green_adv = support_adv  # 使用更强的绿色证据参与后续接管判断。
    if not core_support_ok(core_count, support_count, green_adv):  # 判断最终亮芯是否缺少绿色身份支撑。
        g_lost_count = g_lost_count + 1  # 增加丢失计数但保留上一帧位置。
        if g_last_x >= 0 and g_last_y >= 0 and g_lost_count <= cfg("HOLD_LAST_MAX_LOST", 1):  # 判断是否允许短暂保持上一帧坐标。
            return hold_last_result()  # 输出上一帧坐标，避免黑框边缘光晕接管。
        return invalid_result(0, "support", green_adv, score)  # 输出无效并记录绿色支撑不足。
    if jump_from_last(cx, cy):  # 判断候选是否发生超过配置阈值的突跳。
        if green_adv < cfg("GREEN_JUMP_ACCEPT_ADV", 22) or core_count < cfg("GREEN_JUMP_ACCEPT_CORE", 1):  # 突跳候选必须同时有强绿色证据和亮芯命中。
            local = local_green_recapture(img)  # 尝试在上一帧附近找黑框内弱绿点。
            if local:  # 判断本地重捕获是否成功。
                cx, cy, local_count, green_adv = local  # 使用本地重捕获中心替代突跳候选。
                core_count = 0  # 本地重捕获不使用白芯计数。
                core_roi = None  # 本地重捕获不绘制白芯搜索框。
                blob = None  # 不显示被拒绝的突跳候选框。
                score = green_adv_score(green_adv) + 20  # 给本地重捕获生成保守置信度基础分。
            else:  # 处理本地重捕获失败。
                g_lost_count = g_lost_count + 1  # 增加丢失计数但保留上一帧位置。
                return invalid_result(0, "jump", green_adv, score)  # 输出无效，避免黑框边缘接管坐标。
    g_last_x = cx  # 保存绿点 X 坐标用于下一帧 ROI。
    g_last_y = cy  # 保存绿点 Y 坐标用于下一帧 ROI。
    g_lost_count = 0  # 清零连续丢失计数。
    conf = clamp(score + min(core_count, 20), 1, 100)  # 将候选得分和白芯命中数合成为 1 到 100 的置信度。
    return {"valid": 1, "x": cx, "y": cy, "conf": conf, "err": 0, "roi": g_last_roi, "core_roi": core_roi, "core": core_count, "full": g_last_full, "lost": g_lost_count, "blob": blob, "adv": green_adv, "score": score, "reject": "lab"}  # 返回有效绿点结果并保留调试字段。

def draw_overlay(img, result):  # 绘制绿点调试叠加信息。
    if config.DEBUG_DRAW_DETAIL and result["roi"]:  # 判断是否需要绘制当前 ROI。
        img.draw_rect(result["roi"][0], result["roi"][1], result["roi"][2], result["roi"][3], image.COLOR_BLUE)  # 绘制当前 ROI 框。
    if config.DEBUG_DRAW_DETAIL and result["blob"]:  # 判断是否需要绘制最终候选框。
        x, y, w, h = blob_rect(result["blob"])  # 读取最终候选矩形。
        img.draw_rect(x, y, w, h, image.COLOR_GREEN)  # 绘制最终候选框。
    if config.DEBUG_DRAW_DETAIL and result["core_roi"]:  # 判断是否需要绘制白芯搜索框。
        img.draw_rect(result["core_roi"][0], result["core_roi"][1], result["core_roi"][2], result["core_roi"][3], image.COLOR_BLUE)  # 绘制白芯搜索框便于现场调参。
    if result["valid"]:  # 判断当前绿点是否有效。
        img.draw_rect(result["x"] - config.CENTER_BOX_HALF, result["y"] - config.CENTER_BOX_HALF, config.CENTER_BOX_HALF * 2, config.CENTER_BOX_HALF * 2, image.COLOR_GREEN)  # 按配置绘制绿点中心小框。
    line = "g adv={} score={} rej={}".format(result.get("adv", 0), result.get("score", 0), result.get("reject", "na"))  # 生成绿点候选诊断文本。
    img.draw_string(2, 2 + config.DEBUG_TEXT_STEP * 6, line, image.COLOR_GREEN, scale=config.DEBUG_TEXT_SCALE, wrap=False)  # 在底部调试行显示绿点拒绝原因和评分。
