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

def blob_core_roi(blob):  # 计算亮芯精定位使用的小搜索框。
    x, y, w, h = blob_rect(blob)  # 读取候选绿色外环矩形。
    pad = config.CORE_ROI_PAD  # 读取亮芯搜索框外扩边距。
    cx, cy = blob_center(blob)  # 读取候选质心作为搜索框中心。
    side = clamp(max(w, h) + pad * 2, 1, config.CORE_ROI_MAX_SIDE)  # 计算并限制亮芯搜索框边长。
    half = side // 2  # 计算搜索框半边长。
    x0 = clamp(cx - half, 0, config.TRACK_IMG_W - side)  # 计算并限制搜索框左边界。
    y0 = clamp(cy - half, 0, config.TRACK_IMG_H - side)  # 计算并限制搜索框上边界。
    x1 = x0 + side  # 计算搜索框右边界。
    y1 = y0 + side  # 计算搜索框下边界。
    return [x0, y0, x1 - x0, y1 - y0]  # 返回 MaixPy 矩形格式的亮芯搜索框。

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
        return False  # 亮度不足时不是亮芯。
    if r + g + b < config.GREEN_CORE_MIN_BRIGHT:  # 判断总亮度是否不足。
        return False  # 总亮度不足时不是亮芯。
    if g - r < config.GREEN_CORE_MIN_GREEN_DELTA and g - b < config.GREEN_CORE_MIN_GREEN_DELTA:  # 判断是否缺少绿色激光的轻微偏绿。
        return False  # 纯白或灰白反光不作为绿点中心。
    return True  # 该像素可参与亮芯中心计算。

def refine_core_center(img, blob):  # 在绿色候选附近用亮芯像素精定位中心。
    roi = blob_core_roi(blob)  # 计算亮芯搜索区域。
    sum_x = 0  # 累计亮芯像素的加权 X 坐标。
    sum_y = 0  # 累计亮芯像素的加权 Y 坐标。
    sum_w = 0  # 累计亮芯像素亮度权重。
    count = 0  # 统计命中的亮芯像素数量。
    for y in range(roi[1], roi[1] + roi[3]):  # 遍历亮芯搜索框内的每一行。
        for x in range(roi[0], roi[0] + roi[2]):  # 遍历亮芯搜索框内的每一列。
            r, g, b = pixel_rgb(img, x, y)  # 读取当前像素 RGB。
            if core_pixel_ok(r, g, b):  # 判断当前像素是否属于亮芯。
                weight = r + g + b  # 使用 RGB 总亮度作为中心权重。
                sum_x = sum_x + x * weight  # 累计加权 X 坐标。
                sum_y = sum_y + y * weight  # 累计加权 Y 坐标。
                sum_w = sum_w + weight  # 累计权重。
                count = count + 1  # 累计亮芯像素数量。
    if count <= 0 or sum_w <= 0:  # 判断是否没有找到可靠亮芯。
        cx, cy = blob_center(blob)  # 回退到绿色候选质心。
        return cx, cy, 0, roi  # 返回回退中心和亮芯命中数。
    return int(sum_x / sum_w), int(sum_y / sum_w), count, roi  # 返回亮度加权亮芯中心和命中数。

def ratio10(w, h):  # 计算候选框宽高比乘以十。
    if h <= 0 or w <= 0:  # 判断候选尺寸是否非法。
        return 999  # 返回一个极大比值表示非法候选。
    if w > h:  # 判断宽是否大于高。
        return int(w * 10 / h)  # 返回宽高比乘以十。
    return int(h * 10 / w)  # 返回高宽比乘以十。

def candidate_ok(blob):  # 判断候选 blob 是否像绿色激光点。
    x, y, w, h = blob_rect(blob)  # 读取候选矩形。
    area = w * h  # 计算候选框面积。
    if area < config.MIN_AREA or area > config.MAX_AREA:  # 判断候选面积是否在合理范围外。
        return False  # 面积不合理则丢弃。
    if w > config.MAX_SIDE or h > config.MAX_SIDE:  # 判断候选边长是否过大。
        return False  # 边长过大则丢弃。
    if ratio10(w, h) > config.MAX_RATIO10:  # 判断候选是否过扁。
        return False  # 形状过扁则丢弃。
    return True  # 候选通过基础过滤。

def distance_score(cx, cy):  # 计算候选与上一帧位置的连续性得分。
    if g_last_x < 0 or g_last_y < 0 or g_lost_count >= config.LOST_TO_FULL:  # 判断是否没有可用上一帧。
        return 20  # 没有上一帧时给中性连续性分。
    dx = cx - g_last_x  # 计算 X 方向位移。
    dy = cy - g_last_y  # 计算 Y 方向位移。
    dist2 = dx * dx + dy * dy  # 计算平方距离避免开方。
    if dist2 <= 400:  # 判断是否在 20 像素内。
        return 35  # 非常连续给高分。
    if dist2 <= 6400:  # 判断是否在 80 像素内。
        return 20  # 基本连续给中分。
    return 0  # 跳变较大不给连续性分。

def candidate_score(blob):  # 给候选绿点打分。
    x, y, w, h = blob_rect(blob)  # 读取候选矩形。
    cx, cy = blob_center(blob)  # 计算候选中心。
    area = w * h  # 计算候选面积。
    compact = 30 - min(ratio10(w, h), 30)  # 计算形状紧凑得分。
    size = min(area, 120) * 25 // 120  # 计算候选尺寸得分。
    continuity = distance_score(cx, cy)  # 计算连续性得分。
    return compact + size + continuity  # 返回综合分。

def best_blob(blobs):  # 从候选列表中选出最可信绿点。
    best = None  # 初始化最佳候选为空。
    best_score = -1  # 初始化最佳得分为非法值。
    for blob in blobs:  # 遍历所有候选色块。
        if candidate_ok(blob):  # 判断候选是否通过基础过滤。
            score = candidate_score(blob)  # 计算候选得分。
            if score > best_score:  # 判断是否优于当前最佳候选。
                best = blob  # 保存新的最佳候选。
                best_score = score  # 保存新的最佳得分。
    return best, best_score  # 返回最佳候选和得分。

def invalid_result(err_code=0):  # 生成无效绿点结果。
    return {"valid": 0, "x": -1, "y": -1, "conf": 0, "err": err_code, "roi": g_last_roi, "core_roi": None, "core": 0, "full": g_last_full, "lost": g_lost_count, "blob": None}  # 返回无效结果字典。

def detect(img):  # 在一帧 RGB 图像中检测绿色激光点。
    global g_last_x, g_last_y, g_lost_count  # 声明要修改跟踪位置和丢失计数。
    roi = current_roi()  # 获取本帧搜索区域。
    if roi:  # 判断是否使用 ROI 搜索。
        blobs = img.find_blobs(config.GREEN_THRESHOLDS, roi=roi, pixels_threshold=config.MIN_PIXELS, area_threshold=config.MIN_AREA)  # 在 ROI 内寻找绿色色块。
    else:  # 处理全屏搜索。
        blobs = img.find_blobs(config.GREEN_THRESHOLDS, pixels_threshold=config.MIN_PIXELS, area_threshold=config.MIN_AREA)  # 在全屏寻找绿色色块。
    blob, score = best_blob(blobs)  # 从候选中选出最佳绿点。
    if not blob:  # 判断是否没有找到可信绿点。
        g_lost_count = g_lost_count + 1  # 增加连续丢失计数。
        return invalid_result(0)  # 返回无效但无处理错误的结果。
    cx, cy, core_count, core_roi = refine_core_center(img, blob)  # 优先用亮芯亮度加权中心精定位绿点。
    g_last_x = cx  # 保存绿点 X 坐标用于下一帧 ROI。
    g_last_y = cy  # 保存绿点 Y 坐标用于下一帧 ROI。
    g_lost_count = 0  # 清零连续丢失计数。
    conf = clamp(score + min(core_count, 20), 1, 100)  # 将候选得分和亮芯命中数合成为 1 到 100 的置信度。
    return {"valid": 1, "x": cx, "y": cy, "conf": conf, "err": 0, "roi": g_last_roi, "core_roi": core_roi, "core": core_count, "full": g_last_full, "lost": g_lost_count, "blob": blob}  # 返回有效绿点结果。

def draw_overlay(img, result):  # 绘制绿点调试叠加信息。
    if result["roi"]:  # 判断当前是否有 ROI。
        img.draw_rect(result["roi"][0], result["roi"][1], result["roi"][2], result["roi"][3], image.COLOR_BLUE)  # 绘制当前 ROI 框。
    if result["blob"]:  # 判断当前是否有最终候选 blob。
        x, y, w, h = blob_rect(result["blob"])  # 读取最终候选矩形。
        img.draw_rect(x, y, w, h, image.COLOR_GREEN)  # 绘制最终绿点候选框。
    if result["core_roi"]:  # 判断当前是否有亮芯搜索框。
        img.draw_rect(result["core_roi"][0], result["core_roi"][1], result["core_roi"][2], result["core_roi"][3], image.COLOR_BLUE)  # 绘制亮芯搜索框便于现场调参。
    if result["valid"]:  # 判断当前绿点是否有效。
        img.draw_rect(result["x"] - config.CENTER_BOX_HALF, result["y"] - config.CENTER_BOX_HALF, config.CENTER_BOX_HALF * 2, config.CENTER_BOX_HALF * 2, image.COLOR_GREEN)  # 按配置绘制绿点中心小框。
