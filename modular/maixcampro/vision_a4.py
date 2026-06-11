from maix import image  # 导入 MaixPy 图像模块用于 OpenCV 转换和 overlay 颜色常量。
import config  # 导入现场可调参数配置模块。
import math  # 导入角度和距离计算所需的数学函数。

try:  # 尝试导入 MaixCAM 固件内置的 OpenCV 模块。
    import cv2  # 导入 OpenCV 用于二值化、轮廓层级和四边形拟合。
except Exception:  # 兼容板端固件未启用 OpenCV 的情况。
    cv2 = None  # 标记 OpenCV 不可用并让检测函数返回处理错误。

ERR_A4_PROCESS = 6  # 定义 A4 图像处理不可用或异常错误码。

g_last_points = None  # 保存上一帧 A4 中心线四角点用于稳定判定。
g_last_angle10 = 0  # 保存上一帧 A4 顶边角度用于稳定判定。
g_stable_count = 0  # 保存连续稳定候选帧数。

def reset():  # 重置 A4 检测稳定状态。
    global g_last_points, g_last_angle10, g_stable_count  # 声明要修改 A4 稳定判定全局变量。
    g_last_points = None  # 清空上一帧角点。
    g_last_angle10 = 0  # 清空上一帧角度。
    g_stable_count = 0  # 清空连续稳定帧数。

def clamp(value, low, high):  # 把数值限制在指定范围内。
    if value < low:  # 判断数值是否低于下限。
        return low  # 返回下限。
    if value > high:  # 判断数值是否高于上限。
        return high  # 返回上限。
    return value  # 返回原始数值。

def invalid_result(err_code=0):  # 生成无效 A4 检测结果。
    return {"valid": 0, "raw_valid": 0, "points": [0, 0, 0, 0, 0, 0, 0, 0], "angle10": 0, "conf": 0, "err": err_code, "stable": 0, "outer": None, "outer_coarse": None, "inner": None, "center": None, "area": 0, "ratio10": 0, "hole100": 0}  # 返回协议和 overlay 共用的无效结果。

def point_xy(point):  # 把 OpenCV 点转换成普通浮点坐标。
    return float(point[0]), float(point[1])  # 返回 x 和 y 坐标。

def distance(p0, p1):  # 计算两个点之间的欧氏距离。
    dx = p0[0] - p1[0]  # 计算 X 方向差值。
    dy = p0[1] - p1[1]  # 计算 Y 方向差值。
    return math.sqrt(dx * dx + dy * dy)  # 返回像素距离。

def contour_area(contour):  # 计算轮廓面积。
    return abs(cv2.contourArea(contour))  # 返回轮廓面积绝对值。

def contour_point_list(contour):  # 将 OpenCV 轮廓转换为普通点列表。
    points = []  # 创建普通二维点列表。
    for item in contour:  # 遍历轮廓中的每个边界点。
        x, y = point_xy(item[0])  # 取出当前轮廓点坐标。
        points.append([x, y])  # 保存普通二维点。
    return points  # 返回可用于几何计算的点列表。

def approx_quad_points(contour):  # 将真实轮廓近似为四边形角点。
    perimeter = cv2.arcLength(contour, True)  # 计算闭合轮廓周长用于自适应拟合精度。
    if perimeter <= 0:  # 判断轮廓周长是否非法。
        return None  # 非法轮廓不能生成四边形。
    epsilon = perimeter * config.A4_POLY_EPS1000 / 1000.0  # 按配置比例计算 approxPolyDP 误差上限。
    approx = cv2.approxPolyDP(contour, epsilon, True)  # 用真实轮廓拟合四边形候选。
    if len(approx) != 4:  # 判断拟合结果是否不是四个角点。
        return None  # 非四边形轮廓直接丢弃，避免外接矩形假阳性。
    if not cv2.isContourConvex(approx):  # 判断四边形是否为凸四边形。
        return None  # 非凸四边形不能作为黑胶带矩形框。
    points = []  # 创建普通二维角点列表。
    for point in approx:  # 遍历 OpenCV 近似得到的四个点。
        x, y = point_xy(point[0])  # 取出当前点坐标并转换为浮点数。
        points.append([x, y])  # 保存当前普通角点。
    return order_corners(points)  # 返回统一顺序后的四边形角点。

def points_near_edge(points, p0, p1):  # 筛选靠近某条粗边的轮廓点。
    selected = []  # 创建参与当前边拟合的点列表。
    dx = p1[0] - p0[0]  # 计算粗边 X 方向向量。
    dy = p1[1] - p0[1]  # 计算粗边 Y 方向向量。
    length2 = dx * dx + dy * dy  # 计算粗边长度平方。
    if length2 <= 0:  # 判断粗边是否退化。
        return selected  # 退化边不选择任何点。
    max_dist2 = config.A4_EDGE_POINT_MAX_DIST * config.A4_EDGE_POINT_MAX_DIST  # 计算点到边最大距离平方。
    for point in points:  # 遍历所有真实轮廓点。
        t = ((point[0] - p0[0]) * dx + (point[1] - p0[1]) * dy) / length2  # 计算轮廓点在粗边上的投影比例。
        if t < -0.05 or t > 1.05:  # 判断投影是否明显落在当前边段之外。
            continue  # 当前点属于角外或其他边时跳过。
        near_x = p0[0] + t * dx  # 计算当前点在粗边上的最近 X 坐标。
        near_y = p0[1] + t * dy  # 计算当前点在粗边上的最近 Y 坐标。
        ex = point[0] - near_x  # 计算点到粗边最近点的 X 差值。
        ey = point[1] - near_y  # 计算点到粗边最近点的 Y 差值。
        if ex * ex + ey * ey <= max_dist2:  # 判断当前点是否足够靠近粗边。
            selected.append(point)  # 保存该点用于直线拟合。
    return selected  # 返回当前边附近的轮廓点。

def fit_line_from_points(points):  # 用轮廓点拟合一条二维直线。
    count = len(points)  # 读取参与拟合的点数量。
    if count < config.A4_EDGE_MIN_POINTS:  # 判断当前边点数是否不足。
        return None  # 点数不足时放弃精修。
    cx = sum(point[0] for point in points) / count  # 计算拟合点中心 X。
    cy = sum(point[1] for point in points) / count  # 计算拟合点中心 Y。
    sxx = 0.0  # 初始化 X 方差累计值。
    sxy = 0.0  # 初始化 XY 协方差累计值。
    syy = 0.0  # 初始化 Y 方差累计值。
    for point in points:  # 遍历参与拟合的点。
        x = point[0] - cx  # 计算当前点相对中心的 X。
        y = point[1] - cy  # 计算当前点相对中心的 Y。
        sxx = sxx + x * x  # 累加 X 方差项。
        sxy = sxy + x * y  # 累加 XY 协方差项。
        syy = syy + y * y  # 累加 Y 方差项。
    if sxx + syy <= 0:  # 判断所有点是否退化到同一位置。
        return None  # 退化点集无法拟合直线。
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)  # 计算主方向角度。
    return [cx, cy, math.cos(theta), math.sin(theta)]  # 返回直线经过点和单位方向向量。

def line_intersection(line0, line1):  # 计算两条拟合直线交点。
    x0, y0, dx0, dy0 = line0  # 拆出第一条直线参数。
    x1, y1, dx1, dy1 = line1  # 拆出第二条直线参数。
    det = dx0 * dy1 - dy0 * dx1  # 计算两方向向量叉积。
    if abs(det) < 0.001:  # 判断两条线是否接近平行。
        return None  # 平行线没有稳定交点。
    t = ((x1 - x0) * dy1 - (y1 - y0) * dx1) / det  # 计算第一条线上到交点的比例。
    return [x0 + t * dx0, y0 + t * dy0]  # 返回两条直线交点。

def offset_outer_quad(points, offset_px):  # 将外框四条边沿远离中心方向整体外移。
    if offset_px <= 0:  # 判断是否不需要外移补偿。
        return points  # 不补偿时直接返回原外框。
    cx = sum(point[0] for point in points) / 4.0  # 计算外框中心 X。
    cy = sum(point[1] for point in points) / 4.0  # 计算外框中心 Y。
    lines = []  # 创建外移后的四条边线列表。
    for index in range(4):  # 遍历外框上右下左四条边。
        p0 = points[index]  # 读取当前边起点。
        p1 = points[(index + 1) % 4]  # 读取当前边终点。
        dx = p1[0] - p0[0]  # 计算当前边 X 方向。
        dy = p1[1] - p0[1]  # 计算当前边 Y 方向。
        length = math.sqrt(dx * dx + dy * dy)  # 计算当前边长度。
        if length <= 0:  # 判断当前边是否退化。
            return points  # 边退化时回退原外框。
        ux = dx / length  # 计算当前边单位方向 X。
        uy = dy / length  # 计算当前边单位方向 Y。
        nx = -uy  # 计算当前边的一个单位法线 X。
        ny = ux  # 计算当前边的一个单位法线 Y。
        mx = (p0[0] + p1[0]) / 2.0  # 计算当前边中点 X。
        my = (p0[1] + p1[1]) / 2.0  # 计算当前边中点 Y。
        if nx * (mx - cx) + ny * (my - cy) < 0:  # 判断法线是否指向四边形内部。
            nx = -nx  # 翻转法线到远离中心方向。
            ny = -ny  # 翻转法线到远离中心方向。
        lines.append([p0[0] + nx * offset_px, p0[1] + ny * offset_px, ux, uy])  # 保存外移后的边线。
    shifted = []  # 创建外移后重新求交得到的角点列表。
    pairs = [(3, 0), (0, 1), (1, 2), (2, 3)]  # 定义左上、右上、右下、左下对应的相邻外移边组合。
    for left_index, right_index in pairs:  # 遍历四个角点对应的边线组合。
        point = line_intersection(lines[left_index], lines[right_index])  # 计算相邻外移边线交点。
        if point is None:  # 判断外移边线是否无法稳定求交。
            return points  # 求交失败时回退原外框。
        shifted.append(point)  # 保存外移后的当前角点。
    return order_corners(shifted)  # 返回统一顺序后的外移外框角点。

def refined_quad_points(contour, coarse):  # 基于粗四边形对四条边做直线精修。
    boundary = contour_point_list(contour)  # 提取真实轮廓边界点。
    lines = []  # 创建四条拟合边线列表。
    for index in range(4):  # 逐条处理上右下左四条边。
        p0 = coarse[index]  # 读取当前粗边起点。
        p1 = coarse[(index + 1) % 4]  # 读取当前粗边终点。
        selected = points_near_edge(boundary, p0, p1)  # 选择靠近当前粗边的真实轮廓点。
        line = fit_line_from_points(selected)  # 使用真实轮廓点拟合当前边线。
        if line is None:  # 判断当前边是否无法稳定拟合。
            return coarse  # 精修失败时回退粗四边形，避免直接丢失目标。
        lines.append(line)  # 保存当前拟合边线。
    refined = []  # 创建精修四角点列表。
    pairs = [(3, 0), (0, 1), (1, 2), (2, 3)]  # 定义左上、右上、右下、左下对应的相邻边组合。
    for left_index, right_index in pairs:  # 逐个求四个角点。
        point = line_intersection(lines[left_index], lines[right_index])  # 求相邻两条边线的交点。
        if point is None:  # 判断交点是否不稳定。
            return coarse  # 交点失败时回退粗四边形。
        refined.append(point)  # 保存当前精修角点。
    for index in range(4):  # 检查精修角点是否偏离粗角点过多。
        if distance(refined[index], coarse[index]) > config.A4_REFINE_SHIFT_MAX:  # 判断当前角点是否被噪声拉飞。
            return coarse  # 偏移过大时回退粗四边形。
    return order_corners(refined)  # 返回统一顺序后的精修四边形角点。

def quad_aspect10(points):  # 根据四边形四条边计算长宽比乘以十。
    top = distance(points[0], points[1])  # 计算上边长度。
    right = distance(points[1], points[2])  # 计算右边长度。
    bottom = distance(points[2], points[3])  # 计算下边长度。
    left = distance(points[3], points[0])  # 计算左边长度。
    width = (top + bottom) / 2.0  # 计算上下边平均宽度。
    height = (right + left) / 2.0  # 计算左右边平均高度。
    short_side = min(width, height)  # 计算短边长度。
    long_side = max(width, height)  # 计算长边长度。
    if short_side <= 0:  # 判断短边是否非法。
        return 999  # 返回极大值表示非法长宽比。
    return int(long_side * 10 / short_side)  # 返回四边形长宽比乘以十。

def order_corners(points):  # 将四角点统一为左上附近起点的顺时针顺序。
    cx = sum(point[0] for point in points) / 4.0  # 计算四角点中心 X。
    cy = sum(point[1] for point in points) / 4.0  # 计算四角点中心 Y。
    ordered = sorted(points, key=lambda point: math.atan2(point[1] - cy, point[0] - cx))  # 按图像坐标系角度排序为顺时针。
    start_index = 0  # 初始化左上附近角点索引。
    best_value = ordered[0][0] + ordered[0][1]  # 初始化左上判定值。
    for index in range(1, 4):  # 遍历剩余三个角点。
        value = ordered[index][0] + ordered[index][1]  # 计算越小越接近图像左上方向的判定值。
        if value < best_value:  # 判断当前角点是否更靠近左上方向。
            start_index = index  # 更新起点索引。
            best_value = value  # 更新最优判定值。
    return ordered[start_index:] + ordered[:start_index]  # 返回从左上附近开始的顺时针角点。

def centerline_points(outer, inner):  # 根据外框和内孔角点计算黑胶带中心线。
    points = []  # 创建中心线角点列表。
    for index in range(4):  # 遍历四个对应角点。
        x = (outer[index][0] + inner[index][0]) / 2.0  # 计算中心线 X 坐标。
        y = (outer[index][1] + inner[index][1]) / 2.0  # 计算中心线 Y 坐标。
        points.append([x, y])  # 保存中心线角点。
    return points  # 返回中心线四角点。

def flatten_points(points):  # 把四个点压平成协议使用的一维整数列表。
    flat = []  # 创建协议坐标列表。
    for point in points:  # 遍历四个角点。
        flat.append(int(point[0] + 0.5))  # 保存四舍五入后的 X 坐标。
        flat.append(int(point[1] + 0.5))  # 保存四舍五入后的 Y 坐标。
    return flat  # 返回 x0,y0,x1,y1,x2,y2,x3,y3。

def flat_to_points(flat):  # 把协议一维角点恢复为二维点列表。
    return [[flat[0], flat[1]], [flat[2], flat[3]], [flat[4], flat[5]], [flat[6], flat[7]]]  # 返回 overlay 使用的二维角点。

def edge_angle10(points):  # 计算中心线顶边角度，单位为 0.1 度。
    dx = points[1][0] - points[0][0]  # 计算顶边 X 方向差值。
    dy = points[1][1] - points[0][1]  # 计算顶边 Y 方向差值。
    angle = math.atan2(dy, dx) * 180.0 / math.pi  # 计算图像坐标系中的角度。
    if angle < 0:  # 判断角度是否为负。
        angle = angle + 360.0  # 转换为 0 到 360 度范围。
    return int(angle * 10.0 + 0.5)  # 返回角度乘以十的整数值。

def angle_diff10(a0, a1):  # 计算两个 0.1 度角度的最小差值。
    diff = abs(a0 - a1) % 3600  # 计算环形角度差。
    if diff > 1800:  # 判断是否跨过 0 度边界。
        diff = 3600 - diff  # 折返为最小角度差。
    return diff  # 返回最小角度差。

def max_point_shift(points, last_points):  # 计算当前角点相对上一帧的最大位移。
    if last_points is None:  # 判断是否没有上一帧角点。
        return 9999  # 返回极大位移表示无法比较。
    shift = 0  # 初始化最大位移。
    for index in range(4):  # 遍历四个角点。
        dist = distance(points[index], last_points[index])  # 计算当前角点位移。
        if dist > shift:  # 判断是否刷新最大位移。
            shift = dist  # 更新最大位移。
    return int(shift + 0.5)  # 返回四舍五入后的最大位移。

def candidate_conf(ratio10, hole100):  # 根据形状指标生成置信度。
    aspect_score = 30 - abs(ratio10 - 14) * 5  # 根据 A4 长宽比接近程度计算得分。
    hole_score = 30 - abs(hole100 - config.A4_HOLE_RATIO_TARGET100)  # 根据内孔面积占比接近程度计算得分。
    score = 35 + max(0, aspect_score) + max(0, hole_score)  # 合成基础分、长宽比分和内孔比分。
    return clamp(score, 1, 100)  # 限制置信度在 1 到 100。

def candidate_from_pair(contours, outer_index, inner_index):  # 根据外轮廓和内孔轮廓生成 A4 候选。
    outer_area = contour_area(contours[outer_index])  # 计算外轮廓面积。
    inner_area = contour_area(contours[inner_index])  # 计算内孔轮廓面积。
    if outer_area < config.A4_OUTER_AREA_MIN or outer_area > config.A4_OUTER_AREA_MAX:  # 判断外框面积是否合理。
        return None  # 外框面积不合理时丢弃候选。
    if inner_area <= 0 or inner_area >= outer_area:  # 判断内孔面积是否非法。
        return None  # 内孔非法时丢弃候选。
    hole100 = int(inner_area * 100 / outer_area)  # 计算内孔面积占外框面积百分比。
    if hole100 < config.A4_HOLE_RATIO_MIN100 or hole100 > config.A4_HOLE_RATIO_MAX100:  # 判断内孔占比是否合理。
        return None  # 内孔占比不合理时丢弃候选。
    outer = approx_quad_points(contours[outer_index])  # 将外轮廓拟合为真实四边形粗角点。
    if outer is None:  # 判断外轮廓是否无法稳定拟合成四边形。
        return None  # 外框不是四边形时丢弃候选。
    outer_coarse = outer  # 保存外框粗四边形供 overlay 对比，不参与最终输出。
    inner = approx_quad_points(contours[inner_index])  # 将内孔轮廓拟合为真实四边形粗角点。
    if inner is None:  # 判断内孔轮廓是否无法稳定拟合成四边形。
        return None  # 内孔不是四边形时丢弃候选。
    outer = refined_quad_points(contours[outer_index], outer)  # 对外框四条边做真实轮廓直线精修。
    inner = refined_quad_points(contours[inner_index], inner)  # 对内孔四条边做真实轮廓直线精修。
    outer = offset_outer_quad(outer, config.A4_OUTER_LINE_OFFSET_PX)  # 对外框四条边做向外补偿，抵消外边界偏内。
    ratio10 = quad_aspect10(outer)  # 根据真实外框四边形计算长宽比。
    if ratio10 < config.A4_ASPECT_MIN10 or ratio10 > config.A4_ASPECT_MAX10:  # 判断是否接近 A4 长宽比。
        return None  # 长宽比不合理时丢弃候选。
    center = centerline_points(outer, inner)  # 计算黑胶带中心线四角点。
    angle10 = edge_angle10(center)  # 计算中心线顶边角度。
    conf = candidate_conf(ratio10, hole100)  # 计算候选置信度。
    return {"valid": 0, "raw_valid": 1, "points": flatten_points(center), "angle10": angle10, "conf": conf, "err": 0, "stable": 0, "outer": outer, "outer_coarse": outer_coarse, "inner": inner, "center": center, "area": int(outer_area), "ratio10": ratio10, "hole100": hole100}  # 返回可稳定判定的 A4 候选。

def find_best_candidate(contours, hierarchy):  # 从轮廓层级中寻找最佳黑色矩形环。
    if hierarchy is None:  # 判断是否没有轮廓层级。
        return None  # 没有层级时无有效候选。
    best = None  # 初始化最佳候选为空。
    best_score = -1  # 初始化最佳候选评分。
    rows = hierarchy[0]  # 取出 OpenCV 层级数组。
    for outer_index in range(len(contours)):  # 遍历所有可能的外轮廓。
        if int(rows[outer_index][3]) != -1:  # 判断当前轮廓是否不是顶层外轮廓。
            continue  # 只使用顶层轮廓作为黑胶带外框。
        child_index = int(rows[outer_index][2])  # 读取第一个内孔子轮廓索引。
        while child_index >= 0:  # 遍历当前外轮廓下的所有内孔轮廓。
            candidate = candidate_from_pair(contours, outer_index, child_index)  # 生成当前外框和内孔候选。
            if candidate:  # 判断当前候选是否有效。
                score = candidate["conf"] + candidate["area"] // 1000  # 使用置信度和面积共同选择最可信候选。
                if score > best_score:  # 判断当前候选是否优于最佳候选。
                    best = candidate  # 保存当前最佳候选。
                    best_score = score  # 保存当前最佳候选评分。
            child_index = int(rows[child_index][0])  # 切换到同级下一个内孔轮廓。
    return best  # 返回最佳候选或空。

def apply_stability(candidate):  # 对 A4 候选应用多帧稳定判定。
    global g_last_points, g_last_angle10, g_stable_count  # 声明要修改稳定判定全局变量。
    points = flat_to_points(candidate["points"])  # 转换当前候选角点用于比较。
    shift = max_point_shift(points, g_last_points)  # 计算角点最大位移。
    diff10 = angle_diff10(candidate["angle10"], g_last_angle10)  # 计算角度变化。
    if g_last_points is not None and shift <= config.A4_STABLE_SHIFT_MAX and diff10 <= config.A4_STABLE_ANGLE10_MAX:  # 判断当前候选是否延续上一帧。
        g_stable_count = g_stable_count + 1  # 增加连续稳定帧数。
    else:  # 处理首帧或候选跳变。
        g_stable_count = 1  # 重启稳定计数。
    g_last_points = points  # 保存当前角点用于下一帧比较。
    g_last_angle10 = candidate["angle10"]  # 保存当前角度用于下一帧比较。
    candidate["stable"] = g_stable_count  # 写入当前稳定帧数用于 overlay。
    if g_stable_count >= config.A4_STABLE_FRAMES:  # 判断是否达到锁存所需稳定帧数。
        candidate["valid"] = 1  # 标记当前 A4 结果可被 LOCK_A4 锁存。
    else:  # 处理尚未稳定的候选。
        candidate["valid"] = 0  # 标记当前 A4 候选暂不可锁存。
    return candidate  # 返回写入稳定信息后的候选。

def make_mask(img):  # 将 MaixPy 图像转换为黑色胶带二值图。
    cv_img = image.image2cv(img, ensure_bgr=True, copy=False)  # 将 MaixPy 图像转换为 OpenCV BGR 图像。
    if config.A4_IMG_W == config.SPOT_IMG_W and config.A4_IMG_H == config.SPOT_IMG_H:  # 判断 A4 测试版处理尺寸是否等于相机输入尺寸。
        small = cv_img  # 尺寸相同时直接使用原始 640x480 图像，避免无意义 resize 开销。
    else:  # 处理 A4 检测尺寸低于相机输入的情况。
        small = cv2.resize(cv_img, (config.A4_IMG_W, config.A4_IMG_H))  # 将固定 640x480 输入缩放到 A4 检测尺寸。
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)  # 转换为灰度图。
    _, mask = cv2.threshold(gray, config.A4_BLACK_MAX_GRAY, 255, cv2.THRESH_BINARY_INV)  # 提取低灰度黑色区域。
    if config.A4_MORPH_KERNEL > 1:  # 判断是否需要形态学处理。
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (config.A4_MORPH_KERNEL, config.A4_MORPH_KERNEL))  # 创建矩形形态学核。
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算连接胶带边缘小断点。
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 开运算去除孤立黑色噪点。
    return mask  # 返回黑色区域二值图。

def find_contours(mask):  # 兼容不同 OpenCV 版本的轮廓查找返回值。
    found = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # 使用完整轮廓点寻找黑色矩形环，便于边线精修。
    if len(found) == 3:  # 判断是否为 OpenCV 3 风格返回值。
        return found[1], found[2]  # 返回 contours 和 hierarchy。
    return found[0], found[1]  # 返回 OpenCV 4 风格 contours 和 hierarchy。

def clear_stability():  # 清空稳定计数但保留函数语义清晰。
    reset()  # 调用统一重置函数。

def detect(img):  # 在一帧 A4 测试版处理图像中检测 A4 黑色胶带闭合框。
    if cv2 is None:  # 判断 OpenCV 是否不可用。
        clear_stability()  # 清空稳定状态，避免旧结果被锁存。
        return invalid_result(ERR_A4_PROCESS)  # 返回处理错误。
    mask = make_mask(img)  # 生成黑色二值图。
    contours, hierarchy = find_contours(mask)  # 查找黑色区域轮廓和层级。
    candidate = find_best_candidate(contours, hierarchy)  # 从轮廓层级中寻找 A4 矩形环。
    if not candidate:  # 判断本帧是否没有可靠候选。
        clear_stability()  # 清空稳定状态。
        return invalid_result(0)  # 返回无效但无处理错误。
    return apply_stability(candidate)  # 返回经过多帧稳定判定的结果。

def draw_poly(img, points, color):  # 在 overlay 中绘制四边形。
    if not points:  # 判断是否没有点可画。
        return  # 无点时直接返回。
    scale_x = config.SPOT_IMG_W / config.A4_IMG_W  # 计算 A4 协议坐标到当前显示图像的 X 缩放比例。
    scale_y = config.SPOT_IMG_H / config.A4_IMG_H  # 计算 A4 协议坐标到当前显示图像的 Y 缩放比例。
    for index in range(4):  # 遍历四条边。
        p0 = points[index]  # 读取当前边起点。
        p1 = points[(index + 1) % 4]  # 读取当前边终点。
        img.draw_line(int(p0[0] * scale_x), int(p0[1] * scale_y), int(p1[0] * scale_x), int(p1[1] * scale_y), color)  # 绘制缩放后的当前边。

def draw_points(img, points, color):  # 在 overlay 中绘制中心线角点。
    if not points:  # 判断是否没有点可画。
        return  # 无点时直接返回。
    scale_x = config.SPOT_IMG_W / config.A4_IMG_W  # 计算 A4 协议坐标到当前显示图像的 X 缩放比例。
    scale_y = config.SPOT_IMG_H / config.A4_IMG_H  # 计算 A4 协议坐标到当前显示图像的 Y 缩放比例。
    half = config.A4_POINT_BOX_HALF  # 读取角点小方框半边长。
    for point in points:  # 遍历所有角点。
        img.draw_rect(int(point[0] * scale_x) - half, int(point[1] * scale_y) - half, half * 2, half * 2, color)  # 绘制缩放后的当前角点小方框。

def draw_overlay(img, result, frame_id, fps10, latency_ms):  # 绘制 A4 黑框检测调试叠加信息。
    draw_poly(img, result["outer_coarse"], image.COLOR_YELLOW)  # 绘制外框粗四边形，用于对比精修是否把边线拉偏。
    draw_poly(img, result["outer"], image.COLOR_BLUE)  # 绘制黑胶带外边界候选。
    draw_poly(img, result["inner"], image.COLOR_GREEN)  # 绘制黑胶带内孔边界候选。
    draw_poly(img, result["center"], image.COLOR_RED)  # 绘制黑胶带中心线四边形。
    draw_points(img, result["center"], image.COLOR_RED)  # 绘制中心线四角点。
    line1 = "A4 id={} fps10={} lat={} v={} c={}".format(frame_id, fps10, latency_ms, result["valid"], result["conf"])  # 生成第一行调试文本。
    line2 = "stable={} ang10={} area={} r10={} h={}".format(result["stable"], result["angle10"], result["area"], result["ratio10"], result["hole100"])  # 生成第二行调试文本。
    img.draw_string(2, 2, line1, image.COLOR_GREEN, scale=config.DEBUG_TEXT_SCALE)  # 绘制 A4 运行状态文本。
    img.draw_string(2, 2 + config.DEBUG_TEXT_STEP, line2, image.COLOR_GREEN, scale=config.DEBUG_TEXT_SCALE)  # 绘制 A4 几何指标文本。
