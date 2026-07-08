#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
跳一跳 电脑端自动脚本（基于图像识别 + 物理模型）

原理与 wangshub/wechat_jump_game 的 ADB 方案相同，只是把两处“设备接口”换成了电脑接口：
    ADB 截屏 (adb shell screencap)   ->  屏幕区域截图 (mss)
    ADB 长按 (adb shell input swipe) ->  鼠标长按    (pynput)w
识别算法：
    1) 颜色掩码定位棋子底部中心；
    2) 从上往下扫描第一块非背景像素，得到下一目标块的顶点 x；
    3) 用等距投影几何 (tan30°) 推出目标中心，算出像素距离 D；
    4) 在几何中心附近找"完美跳跃"白点 F5F5F5：命中则用它校准更准的目标中心，
       并据此判定上一跳是否正中靶心（完美）；白点尺寸恒在 50×50 内，据此过滤误检。
物理模型（根据游戏源码反推）：
    v_z(h) = min(70h, 150)          — 水平初速度
    v_y(h) = min(135+15h, 180)      — 竖直初速度
    g = 720                         — 重力加速度
    t_air(h) = 2·v_y(h)/g           — 空中飞行时间
    x(h) = v_z(h)·t_air(h)          — 游戏内 3D 水平距离
    D = k·x(h)                     — 屏幕像素与 3D 距离的比例关系
    k = √(2/3)·W·736/(414·60)      — 由正交相机参数精确计算（W=游戏窗口短边像素）
    由 D 通过二分法反解完整物理公式得按压时间 h，p = 1000h (ms)，对所有距离均精确
    停稳时间 T_stop = t_air_ms + max(700, 50·x, halo_ms) + 100 (ms)
    其中 halo_ms 来自 30fps 录像实测: combo=2x→480ms, 4x→580ms, 6x→750ms, 8x+→~900ms 封顶

用法（在本目录下）：
    pip install -r requirements.txt
    python jump_pc.py            # 自动跳（默认；自动识别区域；若自动识别失败则手动框选）
    python jump_pc.py test       # 截一帧看识别对不对，会存 debug 标注图

自动识别区域：通过 Windows API 查找标题包含"跳一跳"的窗口，获取窗口**客户区**
（GetClientRect + ClientToScreen，已自动排除标题栏和边框）作为游戏区域。
W（游戏短边像素）= min(宽, 高) − WINDOW_UI_OFFSET，用于物理公式计算 k 值。
（客户区已无窗口外框，WINDOW_UI_OFFSET 默认为 0；若模拟器内部有边栏可设正值扣除。）
匹配预览图会存到 debug/region_match.png。
若自动识别失败（如窗口标题不含"跳一跳"），会回退到手动框选模式。
若需匹配其他窗口标题，修改脚本顶部 WINDOW_TITLE 常量即可。

运行中热键（焦点在游戏窗口即可，全局监听）：
    空格      暂停 / 继续
    d        存一张当前识别 debug 图
    q / Esc  退出

k 值由游戏窗口短边像素 W 通过正交相机公式精确计算，无需手动调整。
"""

import os
import sys
import time
import math
import random
import argparse
from collections import namedtuple

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("缺少 opencv-python，请先运行: pip install -r requirements.txt")
try:
    import mss
except ImportError:
    sys.exit("缺少 mss，请先运行: pip install -r requirements.txt")
try:
    from pynput.mouse import Button, Controller as MouseController
    from pynput import keyboard
except ImportError:
    sys.exit("缺少 pynput，请先运行: pip install -r requirements.txt")

HERE = os.path.dirname(os.path.abspath(__file__))
DEBUG_DIR = os.path.join(HERE, "debug")

# --- 棋子颜色范围（默认皮肤，深蓝灰“小人”），沿用 wangshub 的经验值 ---
# 若你换了棋子皮肤导致识别不到，改这里的 RGB 区间。
PIECE_R = (50, 60)
PIECE_G = (53, 63)
PIECE_B = (95, 110)

# --- 完美跳跃中心白点（下一目标正中心的 F5F5F5 椭圆白点）---
# 上一跳正中目标中心（“完美/Perfect”）后，本次目标块正中心会出现一个椭圆点，
# 颜色恒为精确 0xF5F5F5=(245,245,245)。检测到它 => 上一跳完美，且它就是本次目标的精确中心。
CENTER_DOT_RGB = (245, 245, 245)  # 白点核心恒为精确 F5F5F5，直接精确匹配即可
CENTER_DOT_MAX_SIDE = 50     # 白点包围盒边长恒在 50×50 内：超过它的一律不是白点（滤掉白块顶面/反光条/分数字）
CENTER_DOT_WIN_X = 0.16      # 搜索窗半宽（占区域宽比例），限定在几何中心附近，避免扫到背景浅色
CENTER_DOT_WIN_Y = 0.12      # 搜索窗半高（占区域高比例）

# 识别结果：棋子落脚点、目标中心、目标块顶点、是否完美（白点命中）、白点坐标。
Detection = namedtuple(
    "Detection",
    "piece_x piece_y board_x board_y board_top_x board_top_y perfect dot")

# --- 自动识别区域：通过 Windows 窗口标题查找 ---
WINDOW_TITLE = "跳一跳"        # 查找标题包含此字符串的窗口作为游戏区域
WINDOW_UI_OFFSET = 0          # 客户区已排除标题栏/边框；若模拟器内部还有边栏可设正值扣除

# --- 物理模型参数（根据游戏源码反推）---
# 屏幕像素距离 D 与游戏内 3D 水平距离 x 的关系: D = k·x
# k 由正交相机参数精确计算: k = √(2/3) · W · 736 / (414 · 60)
# 其中 W = 游戏窗口短边像素（自动从窗口尺寸获取，已扣除外层 UI）
# 重力加速度（游戏内单位）
PHYS_G = 720.0
# 竖直初速度 v_y(h) = min(135+15h, 180)
PHYS_VY0 = 135.0            # 竖直初速度基数
PHYS_VY_INC = 15.0          # 竖直初速度增量（每秒）
PHYS_VY_MAX = 180.0         # 竖直初速度上限
# --- 停稳时间模型（全部参数来自游戏源码精确值）---
# 源码落地后并行启动的动画（取最长者）:
#   挤压动画   duration=0.15+0.15=0.3s(300ms) — customAnimation.to body.scale
#   方块回弹   duration=0.5s(500ms) Elastic — block.rebound()
#   粒子飞散   duration=0.3~0.5s(300~500ms) — scatterParticles()
#   得分显示   duration=0.7s(700ms) — showAddScore TweenAnimation 上升+淡出
#     ↑ 得分数字从瓶子位置飘起、淡出，700ms 后才彻底消失。
#       若截图时此动画未完成，白色数字会干扰下一次识别。
#   完美光环   duration≈min(900, 340+140·combo//2)ms — 30fps 录像实测线性拟合
#     ↑ combo=2x→480ms, 4x→620ms, 6x→760ms, 8x+→900ms 封顶
#   场景移动   duration=500·|vars|/10=50·|vars| (ms) — moveGradually()
#              其中 |vars|≤x_curr (当前跳3D距离), 用 x_curr 做保守上界
# T_stop = t_air_ms + max(700, 50·x_curr, halo_ms) + 100
PHYS_SQUEEZE_MS = 300.0         # 落地挤压动画 (ms) — 源码精确值
PHYS_SCORE_DISPLAY_MS = 700.0   # 得分显示动画 (ms) — 源码 showAddScore 精确值
PHYS_SCENE_FACTOR = 50.0        # 场景移动系数 (ms/wu) — 源码 500/10
PHYS_SETTLE_SAFETY_MS = 100.0   # 安全余量 (ms)

# 完美光环耗时 (ms) — 30fps 录像实测拟合: halo_ms ≈ min(900, 340 + 140·combo//2)
# combo=2→480, 4→620, 6→760, 8+→900(封顶); ±40ms 拟合误差, 100ms 余量覆盖
_HALO_BASE = 340.0
_HALO_PER_LAYER = 140.0
_HALO_CAP = 900.0


def _halo_ms(double):
    """根据 combo 倍率返回光环可见时长 (ms)。线性拟合 30fps 录像实测数据。"""
    layers = int(double) // 2
    if layers <= 0:
        return 0.0
    return min(_HALO_CAP, _HALO_BASE + _HALO_PER_LAYER * layers)

# --- 按压抖动（避免被检测为脚本）---
PHYS_JITTER = 0           # 三角分布对称抖动 ±2%

# --- 速度优化：按压与截图识别重叠 ---
# 实测 1000 次跳跃按压时间最小值为 160ms，因此先按压 100ms（<160ms 不触发跳跃）
# 再截图识别，将识别耗时与按压重叠，每次跳跃节省约 70ms。
OVERLAP_MS = 0          # 按压首段时长（ms），在此期间完成截图+识别


def calc_k(W):
    """由游戏窗口短边像素 W，通过正交相机参数精确计算比例系数 k。

    k = √(2/3) · W · 736 / (414 · 60)
    ≈ 0.02418 · W
    """
    return math.sqrt(2.0 / 3.0) * W * 736.0 / (414.0 * 60.0)


def x_of_h(h):
    """完整物理公式：游戏内 3D 水平距离 x(h) = v_z(h)·t_air(h)，含 min 上限。

    v_z(h) = min(70h, 150)
    v_y(h) = min(135+15h, 180)
    t_air(h) = 2·v_y(h)/720
    """
    vz = min(70.0 * h, 150.0)
    vy = min(PHYS_VY0 + PHYS_VY_INC * h, PHYS_VY_MAX)
    t_air = 2.0 * vy / PHYS_G
    return vz * t_air


def calc_press_ms(dist_px, W):
    """根据屏幕像素距离 D 和游戏窗口短边 W，用完整物理模型（含 min 上限）二分反解按压时间。

    D = k·x(h)，其中 x(h) = min(70h,150)·2·min(135+15h,180)/720。
    由于 x(h) 单调递增，直接用二分法求解 h，无需分段解析，对所有距离都精确。
    返回按压毫秒数。
    """
    if dist_px <= 0:
        return 0.0
    k = calc_k(W)
    target_x = dist_px / k
    # 二分搜索 h ∈ [0, 10]，50 次迭代精度 > 1e-15
    lo, hi = 0.0, 10.0
    for _ in range(50):
        mid = (lo + hi) * 0.5
        if x_of_h(mid) < target_x:
            lo = mid
        else:
            hi = mid
    h = (lo + hi) * 0.5
    return h * 1000.0


def calc_settle_ms(press_ms, dist_px=None, W=None, double=1):
    """根据按压毫秒数估算停稳时间（飞行 + 落地后并行动画取最长 + 安全余量）。

    全部参数来自游戏源码精确值，光环时长来自 30fps 录像实测:
      - 得分显示: 700ms (showAddScore)
      - 完美光环: 480~900ms (combo=2x→8x+，实测数据)
      - 场景移动: 50·|vars| ms, |vars|≤x_curr, 用 x_curr 做保守上界
      - 安全余量: 100ms
    若提供 dist_px 和 W, x_curr = dist_px/k(W); 否则由 press_ms 反推。
    double 为游戏内 combo 倍率 (1=非完美, 2/4/6/.../32=连续完美)。
    """
    h = press_ms / 1000.0
    v_y = min(PHYS_VY0 + PHYS_VY_INC * h, PHYS_VY_MAX)
    t_air_ms = 2.0 * v_y / PHYS_G * 1000.0
    # 当前跳的游戏内 3D 水平距离
    if dist_px is not None and W is not None and dist_px > 0:
        x_curr = dist_px / calc_k(W)
    else:
        x_curr = x_of_h(h)
    # 着陆后动画全部并行：得分显示、光环、场景移动取最长
    scene_ms = max(PHYS_SCORE_DISPLAY_MS, _halo_ms(double),
                   PHYS_SCENE_FACTOR * x_curr)
    return t_air_ms + scene_ms + PHYS_SETTLE_SAFETY_MS


def apply_jitter(nominal_ms):
    """对名义按压毫秒数施加三角分布对称抖动（±2%），避免被检测为脚本。

    三角分布中心=1.0，范围 [1-r, 1+r]，中间概率最高。
    返回 (jittered_ms, jitter_factor)。
    """
    r = PHYS_JITTER
    factor = random.triangular(1.0 - r, 1.0 + r, 1.0)
    return nominal_ms * factor, factor


def set_dpi_aware():
    """让截图坐标与鼠标坐标都以物理像素为准，避免高分屏缩放导致点偏。"""
    if sys.platform == "win32":
        try:
            import ctypes
            try:
                ctypes.windll.shcore.SetProcessDpiAwareness(2)  # PER_MONITOR_AWARE
            except Exception:
                ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass

def grab(sct, region):
    """region = [left, top, width, height]，返回 RGB ndarray。"""
    l, t, w, h = region
    raw = np.array(sct.grab({"left": l, "top": t, "width": w, "height": h}))
    return raw[:, :, :3][:, :, ::-1].copy()  # BGRA -> RGB

def select_region():
    """全屏截一张，让用户拖框选出游戏区域，返回 [left, top, w, h]（含显示器偏移）。"""
    with mss.MSS() as sct:
        mon = sct.monitors[1]  # 主显示器
        full = np.array(sct.grab(mon))[:, :, :3]  # BGR
    H, W = full.shape[:2]
    scale = min(1.0, 1280.0 / W, 800.0 / H)
    disp = cv2.resize(full, None, fx=scale, fy=scale) if scale < 1 else full.copy()
    print("拖拽鼠标框住整个游戏画面（棋子 + 目标都要在框内），回车确认，c 取消。")
    r = cv2.selectROI("select game area", disp, showCrosshair=False, fromCenter=False)
    cv2.destroyAllWindows()
    x, y, w, h = r
    if w == 0 or h == 0:
        return None
    inv = 1.0 / scale
    return [int(mon["left"] + x * inv), int(mon["top"] + y * inv),
            int(w * inv), int(h * inv)]

def find_window_by_title(title_substring):
    """通过 Windows API 查找标题包含指定字符串的可见窗口。
    返回 [(left, top, right, bottom, title, hwnd), ...]，坐标为窗口**客户区**的屏幕物理像素
    （已排除标题栏和边框，即 GetClientRect + ClientToScreen）。
    """
    if sys.platform != "win32":
        return []
    import ctypes
    from ctypes import wintypes

    user32 = ctypes.windll.user32
    results = []

    WNDENUMPROC = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)

    def enum_callback(hwnd, lParam):
        if not user32.IsWindowVisible(hwnd):
            return True
        length = user32.GetWindowTextLengthW(hwnd)
        if length == 0:
            return True
        buf = ctypes.create_unicode_buffer(length + 1)
        user32.GetWindowTextW(hwnd, buf, length + 1)
        if title_substring in buf.value:
            # 获取客户区（不含标题栏/边框），并转为屏幕坐标
            cli_rect = wintypes.RECT()
            if user32.GetClientRect(hwnd, ctypes.byref(cli_rect)):
                # GetClientRect 的 (left,top) 恒为 (0,0)，(right,bottom) = 宽高
                pt_tl = wintypes.POINT(0, 0)
                user32.ClientToScreen(hwnd, ctypes.byref(pt_tl))
                pt_br = wintypes.POINT(cli_rect.right, cli_rect.bottom)
                user32.ClientToScreen(hwnd, ctypes.byref(pt_br))
                results.append((pt_tl.x, pt_tl.y, pt_br.x, pt_br.y, buf.value, hwnd))
        return True

    callback = WNDENUMPROC(enum_callback)
    user32.EnumWindows(callback, 0)
    return results


def bring_window_to_front(hwnd):
    """将指定窗口拉到前台并激活（恢复最小化、设为前台窗口）。"""
    if sys.platform != "win32":
        return
    import ctypes
    from ctypes import wintypes
    user32 = ctypes.windll.user32
    SW_RESTORE = 9
    SW_SHOW = 5
    if user32.IsIconic(hwnd):
        user32.ShowWindow(hwnd, SW_RESTORE)
    user32.ShowWindow(hwnd, SW_SHOW)
    user32.SetForegroundWindow(hwnd)


def focus_game_window():
    """查找游戏窗口并将其拉到前台。成功返回 True。"""
    windows = find_window_by_title(WINDOW_TITLE)
    if not windows:
        print(f"未找到标题包含「{WINDOW_TITLE}」的窗口，无法自动聚焦")
        return False
    # 多个匹配：选面积最大的
    windows.sort(key=lambda w: (w[2] - w[0]) * (w[3] - w[1]), reverse=True)
    hwnd = windows[0][5]
    bring_window_to_front(hwnd)
    print(f"已聚焦窗口: {windows[0][4]}")
    return True


def auto_detect_region(save_debug=True):
    """
    通过 Windows API 查找标题含 WINDOW_TITLE 的窗口，自动获取游戏区域。
    返回 (region, info)；失败时 region 为 None、info 为原因字符串。
    """
    title = WINDOW_TITLE
    windows = find_window_by_title(title)
    if not windows:
        return None, f"未找到标题包含「{title}」的窗口，请确认游戏窗口已打开且标题含此关键字"

    if len(windows) > 1:
        # 多个匹配：优先选面积最大的
        windows.sort(key=lambda w: (w[2] - w[0]) * (w[3] - w[1]), reverse=True)
        print(f"找到 {len(windows)} 个匹配窗口，选用面积最大的: {windows[0][4]}")

    left, top, right, bottom, win_title, hwnd = windows[0]
    region = [left, top, right - left, bottom - top]
    info = {"window_title": win_title}

    if region[2] <= 10 or region[3] <= 10:
        return None, f"窗口「{win_title}」尺寸异常 {region}（可能已最小化？）"

    if save_debug:
        os.makedirs(DEBUG_DIR, exist_ok=True)
        with mss.MSS() as sct:
            mon = sct.monitors[0]
            screen = np.array(sct.grab(mon))[:, :, :3].copy()  # BGR, contiguous
        ox, oy = mon["left"], mon["top"]
        cv2.rectangle(screen, (left - ox, top - oy),
                      (right - ox, bottom - oy), (255, 200, 0), 2)
        cv2.putText(screen, f"Window: {win_title}", (left - ox + 5, top - oy + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imwrite(os.path.join(DEBUG_DIR, "region_match.png"), screen)

    return region, info

def detect_region():
    """自动识别游戏区域；失败则回退到手动框选。返回 region 或 sys.exit。"""
    region, info = auto_detect_region()
    if region:
        W = min(region[2], region[3]) - WINDOW_UI_OFFSET
        k = calc_k(W)
        print(f"自动识别区域: {region}  (窗口: {info.get('window_title', '?')}, W={W}, k={k:.3f})")
        return region
    print(f"自动识别失败：{info}")
    print("改用手动框选……")
    region = select_region()
    if not region:
        sys.exit("已取消。")
    W = min(region[2], region[3]) - WINDOW_UI_OFFSET
    print(f"手动区域: {region}, 短边 W={W}")
    return region

def find_center_dot(img_rgb, near_xy, top_ignore, scale):
    """
    在几何目标中心附近找“完美跳跃”留下的 F5F5F5 椭圆白点。
    找到 => 上一跳完美，返回精确中心 (x, y)；否则 None。

    near_xy: 几何法推出的目标中心，用来把搜索限制在它周围，排除背景浅色的误检。
    只在这个窗口里找、颜色精确等于 F5F5F5、包围盒 ≤50×50、形状近椭圆，一起过滤。
    使用 cv2.fitEllipse 对轮廓拟合椭圆取几何中心（比像素质心更精准），
    按面积最大选择候选（白点只有一个，面积最大的就是它）。
    """
    h, w, _ = img_rgb.shape
    nx, ny = near_xy
    half_w = max(14, int(w * CENTER_DOT_WIN_X))
    half_h = max(14, int(h * CENTER_DOT_WIN_Y))
    x0 = max(0, nx - half_w); x1 = min(w, nx + half_w)
    y0 = max(top_ignore, ny - half_h); y1 = min(h, ny + half_h)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None

    roi = img_rgb[y0:y1, x0:x1]
    R, G, B = CENTER_DOT_RGB
    mask = ((roi[:, :, 0] == R) & (roi[:, :, 1] == G) &
            (roi[:, :, 2] == B)).astype(np.uint8)   # 白点核心=精确 F5F5F5，直接精确匹配
    if int(mask.sum()) < 6:
        return None

    # 白点包围盒恒 ≤50×50（用户实测的硬上限，按截图像素计），面积下限滤掉零星噪点。
    max_side = CENTER_DOT_MAX_SIDE
    area_min = max(6, int(20 * scale * scale))

    # 提取轮廓，对每个候选用 fitEllipse 拟合椭圆取几何中心（比像素质心更精确），
    # 按面积最大选择最佳候选（白点只有一个，面积最大的就是它）。
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_area, best_center = 0.0, None
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < area_min:
            continue
        bx, by, bw, bh = cv2.boundingRect(cnt)
        if bw > max_side or bh > max_side or bh == 0:
            continue
        aspect = bw / float(bh)
        if aspect < 0.55 or aspect > 3.2:
            continue
        # fitEllipse 拟合椭圆取几何中心，需要 ≥5 个轮廓点
        if cnt.shape[0] >= 5:
            try:
                ellipse = cv2.fitEllipse(cnt)
                ecx, ecy = ellipse[0]
            except cv2.error:
                ecx, ecy = bx + bw / 2.0, by + bh / 2.0
        else:
            ecx, ecy = bx + bw / 2.0, by + bh / 2.0
        if area > best_area:
            best_area = area
            best_center = (int(round(x0 + ecx)), int(round(y0 + ecy)))

    return best_center

def find_piece_and_board(img_rgb, top_ignore_ratio=0.20):
    """
    返回 Detection 或 None（坐标均相对截取区域的像素）。
    先几何法定位棋子与目标中心，再在目标中心附近找 F5F5F5 白点：
    命中就用白点校准更准的中心，并置 perfect=True（说明上一跳正中靶心）。
    """
    h, w, _ = img_rgb.shape
    scale = w / 1080.0
    piece_half = max(20, int(60 * scale))       # 排除棋子本体的横向半宽
    base_lift = max(5, int(20 * scale))          # 从棋子最底像素上抬到落脚中心

    r = img_rgb[:, :, 0].astype(np.int16)
    g = img_rgb[:, :, 1].astype(np.int16)
    b = img_rgb[:, :, 2].astype(np.int16)
    mask = ((r > PIECE_R[0]) & (r < PIECE_R[1]) &
            (g > PIECE_G[0]) & (g < PIECE_G[1]) &
            (b > PIECE_B[0]) & (b < PIECE_B[1]))
    # 取前两大连通域（棋子身体+头部），滤除零星噪点，避免噪点拉偏水平中心
    n_lbl, lbls, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    areas = [(int(stats[i, cv2.CC_STAT_AREA]), i) for i in range(1, n_lbl)]
    areas.sort(reverse=True)
    top2_labels = {i for _, i in areas[:2]}
    filtered = np.isin(lbls, list(top2_labels))
    ys, xs = np.where(filtered)
    if xs.size < 10:
        return None
    piece_x = int(round(xs.mean()))
    piece_y = max(0, int(ys.max()) - base_lift)

    # 从上往下找下一块的顶点：逐行与该行最左像素（背景）比较，取首个明显差异像素
    # 对棋子掩码做小幅膨胀，确保完全排除棋子边缘像素，避免棋子比目标块高时被误识别
    piece_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    piece_exclusion = cv2.dilate(filtered.astype(np.uint8), piece_kernel, iterations=1).astype(bool)
    top_ignore = int(h * top_ignore_ratio)
    board_top_x, board_top_y = 0, 0
    found = False
    for y in range(top_ignore, int(h * 2 / 3)):
        row = img_rgb[y].astype(np.int16)
        bg = row[0]
        diff = np.abs(row - bg).sum(axis=1)
        cand = np.where(diff > 10)[0]
        # 排除棋子本体所在的所有像素（完整棋子掩码 + 横向安全余量双重过滤）
        cand = cand[~piece_exclusion[y, cand]]
        cand = cand[np.abs(cand - piece_x) > piece_half]
        if cand.size > 0:
            board_top_x = int(round(cand.mean()))
            board_top_y = y
            found = True
            break
    if not found:
        return None

    # 等距投影：目标中心相对棋子在 30° 斜线上（几何法，作为兜底与白点搜索的种子）
    board_x = board_top_x
    board_y = max(0, min(h - 1,
                        int(piece_y - abs(board_x - piece_x) * (math.sqrt(3) / 3.0))))

    # 完美跳跃白点：若上一跳正中目标中心，本块正中心会有 F5F5F5 椭圆白点。
    # 命中就用它做更精确的中心校准，并据此判定“上一跳完美”。
    dot = find_center_dot(img_rgb, (board_x, board_y), top_ignore, scale)
    if dot is not None:
        board_x, board_y = dot

    return Detection(piece_x, piece_y, board_x, board_y,
                     board_top_x, board_top_y, dot is not None, dot)

def annotate(img_rgb, det, dist=None, info_lines=None):
    im = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    if det:
        cv2.circle(im, (det.piece_x, det.piece_y), 8, (0, 255, 0), 2)      # 棋子 绿
        cv2.circle(im, (det.board_x, det.board_y), 8, (0, 0, 255), 2)      # 目标 红
        cv2.line(im, (det.piece_x, det.piece_y),
                 (det.board_x, det.board_y), (255, 200, 0), 2)
        if det.dot is not None:
            cv2.circle(im, det.dot, 5, (255, 0, 255), -1)                  # 完美白点 品红实心
            txt_x = min(det.dot[0] + 8, im.shape[1] - 1)
            txt_y = max(10, det.dot[1] - 8)
            cv2.putText(im, "PERFECT", (txt_x, txt_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        if dist is not None:
            cv2.putText(im, f"dist={dist:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        if info_lines:
            for i, line in enumerate(info_lines):
                cv2.putText(im, line, (10, 60 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    return im

def cmd_test():
    region = detect_region()
    os.makedirs(DEBUG_DIR, exist_ok=True)
    with mss.MSS() as sct:
        img = grab(sct, region)
    det = find_piece_and_board(img)
    if not det:
        cv2.imwrite(os.path.join(DEBUG_DIR, "test.png"),
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print("识别失败：没找到棋子或目标。请确认游戏画面在框内、棋子是默认皮肤。")
        print(f"已存原始截图: {os.path.join(DEBUG_DIR, 'test.png')}")
        return
    px, py, bx, by = det.piece_x, det.piece_y, det.board_x, det.board_y
    dist = math.hypot(bx - px, by - py)
    W = min(region[2], region[3]) - WINDOW_UI_OFFSET
    k = calc_k(W)
    press_ms = calc_press_ms(dist, W)
    settle_ms = calc_settle_ms(press_ms, dist, W)
    out = os.path.join(DEBUG_DIR, "test.png")
    cv2.imwrite(out, annotate(img, det, dist))
    print(f"棋子=({px},{py})  目标=({bx},{by})  距离={dist:.1f}px")
    print(f"W={W} k={k:.3f} -> h={press_ms/1000:.3f}s  按压={press_ms:.0f}ms  停稳(公式)={settle_ms:.0f}ms")
    if det.perfect:
        print(f"检测到中心白点 {det.dot} -> 上一跳【完美】，已用白点校准目标中心"
              f"（几何顶点 x={det.board_top_x} -> 校准后 x={bx}）")
    else:
        print("未检测到中心白点 -> 上一跳非完美（或白点已消失），使用几何投影中心")
    print(f"已存标注图: {out} （绿点=棋子落脚，红点=目标中心，品红=完美白点；不准就调颜色区间或区域）")

class Runner:
    def __init__(self, region):
        self.region = region
        # 游戏窗口短边（已扣除外层 UI），用于精确计算 k
        self.W = min(self.region[2], self.region[3]) - WINDOW_UI_OFFSET
        # k = √(2/3) · W · 736 / (414 · 60)，由正交相机参数精确计算，无需手动调整
        self.k = calc_k(self.W)
        self.mouse = MouseController()
        self.paused = False
        self.stop = False
        self.dump = False
        # 完美连击追踪：模拟游戏内 double 变量 (1=非完美, 2/4/6/.../32)
        self.combo_double = 1
        # 保存上一跳的识别截图与参数，用于非完美跳 debug 存图
        self.prev_img = None       # 上一跳的识别截图 (RGB ndarray)
        self.prev_det = None       # 上一跳的 Detection
        self.prev_dist = None      # 上一跳的像素距离
        self.prev_n = 0            # 上一跳的编号
        self.prev_press_ms = None  # 上一跳的名义按压毫秒
        self.prev_settle_ms = None # 上一跳的停稳耗时毫秒

    def press_point(self):
        l, t, w, h = self.region
        # 点在画面下方空白处，避开按钮；加抖动，避免定点连点被判异常
        x = l + w // 2 + random.randint(-8, 8)
        y = t + int(h * 0.82) + random.randint(-8, 8)
        return x, y

    def jump(self, dist):
        """根据像素距离用物理模型计算按压毫秒数并执行长按。

        返回 (nominal_ms, actual_ms, settle_ms, jitter_factor)。
        """
        nominal_ms = calc_press_ms(dist, self.W)
        actual_ms, jitter_factor = apply_jitter(nominal_ms)
        px, py = self.press_point()
        self.mouse.position = (px, py)
        time.sleep(0.02)
        self.mouse.press(Button.left)
        time.sleep(actual_ms / 1000.0)
        self.mouse.release(Button.left)
        settle_ms = calc_settle_ms(nominal_ms, dist, self.W, self.combo_double)
        return nominal_ms, actual_ms, settle_ms, jitter_factor

    def on_key(self, key):
        try:
            if key == keyboard.Key.space:
                self.paused = not self.paused
                print("[暂停]" if self.paused else "[继续]")
            elif key == keyboard.Key.esc:
                self.stop = True
                return False
            elif hasattr(key, "char") and key.char in ("q", "Q"):
                self.stop = True
                return False
            elif hasattr(key, "char") and key.char in ("d", "D"):
                self.dump = True
        except Exception:
            pass

    def loop(self):
        os.makedirs(DEBUG_DIR, exist_ok=True)
        focus_game_window()
        time.sleep(0.3)  # 等窗口切到前台稳定
        print(f"开始。W={self.W} k={self.k:.3f}（精确公式），空格暂停，d 存图，q 退出。")
        if OVERLAP_MS > 0:
            print(f"速度优化：按压首{OVERLAP_MS}ms重叠截图识别（最小按压>={OVERLAP_MS}ms，安全）")
        else:
            print("速度优化：已禁用（OVERLAP_MS=0，使用传统逐帧识别流程）")
        n = 0
        perfect_total = 0  # 累计"上一跳完美"次数（本块出现白点即计一次）
        loop_prev = None   # 上次识别成功的时间戳，用来看两次识别间隔
        first_jump = True  # 第一跳用传统方式（需先截图才能知道往哪跳）
        with mss.MSS() as sct:
            while not self.stop:
                if self.paused:
                    time.sleep(0.15)
                    continue

                t_press_start = None  # 重叠模式下按压开始时间戳

                # ── 重叠优化：非第一跳时，先开始按压（<160ms 不触发跳跃），再截图识别 ──
                if not first_jump:
                    px, py = self.press_point()
                    self.mouse.position = (px, py)
                    time.sleep(0.02)
                    self.mouse.press(Button.left)
                    t_press_start = time.perf_counter()
                    # 按压 OVERLAP_MS 积累按压量，同时为截图+识别留出窗口
                    time.sleep(OVERLAP_MS / 1000.0)

                # ── 截图 + 识别（传统模式：按压前；重叠模式：按压期间，画面尚未变化）──
                t_grab0 = time.perf_counter()
                img = grab(sct, self.region)
                t_grab1 = time.perf_counter()
                det = find_piece_and_board(img)
                t_rec1 = time.perf_counter()

                if not det:
                    if not first_jump:
                        self.mouse.release(Button.left)  # 释放已按下的鼠标
                    print("识别失败，重试…（可能在动画中/皮肤不符）")
                    time.sleep(0.6)
                    continue

                px, py, bx, by = det.piece_x, det.piece_y, det.board_x, det.board_y
                dist = math.hypot(bx - px, by - py)

                if self.dump:
                    cv2.imwrite(os.path.join(DEBUG_DIR, f"frame_{n:04d}.png"),
                                annotate(img, det, dist))
                    self.dump = False

                n += 1
                # 完美连击追踪：模拟游戏内 double (源码 showAddScore: 首次完美→2, 连续→+2, 最大32, 非完美→1)
                if n > 1 and det.perfect:
                    perfect_total += 1
                    self.combo_double = 2 if self.combo_double == 1 else min(self.combo_double + 2, 32)
                elif n > 1:
                    self.combo_double = 1

                # 物理计算
                press_ms = calc_press_ms(dist, self.W)
                settle_ms = calc_settle_ms(press_ms, dist, self.W, self.combo_double)
                grab_ms = (t_grab1 - t_grab0) * 1000.0
                rec_ms = (t_rec1 - t_grab1) * 1000.0
                total_ms = (t_rec1 - t_grab0) * 1000.0

                # ── 日志与调试 ──
                since_prev = ("" if loop_prev is None
                              else f" 距上次识别={(t_rec1 - loop_prev) * 1000.0:.0f}ms")
                loop_prev = t_rec1

                # 非完美跳 debug 存图
                if n > 1 and not det.perfect and self.prev_img is not None:
                    info_lines = [
                        f"Jump #{self.prev_n}  [NOT PERFECT]",
                        f"Time: {time.strftime('%H:%M:%S')}",
                        f"Distance: {self.prev_dist:.1f} px",
                        f"W: {self.W}  k: {self.k:.3f}",
                        f"Press(nom): {self.prev_press_ms:.0f} ms",
                        f"Settle(est): {self.prev_settle_ms:.0f} ms",
                        f"Piece: ({self.prev_det.piece_x}, {self.prev_det.piece_y})",
                        f"Board: ({self.prev_det.board_x}, {self.prev_det.board_y})",
                        f"BoardTop: ({self.prev_det.board_top_x}, {self.prev_det.board_top_y})",
                        f"WhiteDot: NO",
                    ]
                    cv2.imwrite(
                        os.path.join(DEBUG_DIR, f"non_perfect_{self.prev_n:04d}.png"),
                        annotate(self.prev_img, self.prev_det, self.prev_dist, info_lines))

                # 保存当前帧为上一跳状态
                self.prev_img = img.copy()
                self.prev_det = det
                self.prev_dist = dist
                self.prev_n = n
                self.prev_press_ms = press_ms
                self.prev_settle_ms = settle_ms

                # 日志输出
                dot_seg = f"白点=有{det.dot}" if det.perfect else "白点=无"
                combo_seg = f"combo={self.combo_double}x halo={_halo_ms(self.combo_double):.0f}ms"
                if n == 1:
                    perfect_seg = f" {dot_seg}(首帧,无上一跳可判)"
                elif det.perfect:
                    perfect_seg = (f" {dot_seg} 上跳=完美 校准x{det.board_top_x}->{bx} "
                                   f"完美{perfect_total}/{n - 1} {combo_seg}")
                else:
                    perfect_seg = f" {dot_seg} 上跳=偏 完美{perfect_total}/{n - 1} {combo_seg}"
                h_sec = press_ms / 1000.0
                if t_press_start is not None:
                    overlap_info = f" 重叠已按={(t_rec1 - t_press_start) * 1000.0:.0f}ms"
                else:
                    overlap_info = ""
                print(f"#{n} [{time.strftime('%H:%M:%S')}.{int((t_rec1 % 1) * 1000):03d}] "
                      f"D={dist:.1f}px W={self.W} k={self.k:.3f} h={h_sec:.3f}s "
                      f"按压(名义)={press_ms:.0f}ms 抖动±{PHYS_JITTER*100:.0f}% "
                      f"停稳(公式)={settle_ms:.0f}ms "
                      f"截图={grab_ms:.0f}ms 识别={rec_ms:.0f}ms 合计={total_ms:.0f}ms"
                      f"{since_prev}{overlap_info}{perfect_seg}")

                # ── 执行跳跃 ──
                if first_jump or OVERLAP_MS <= 0:
                    # 传统方式：按压全过程
                    nominal_ms, actual_ms, actual_settle_ms, jitter_factor = self.jump(dist)
                    time.sleep(actual_settle_ms / 1000.0)
                    time.sleep(0.05)
                    if OVERLAP_MS > 0:
                        first_jump = False
                else:
                    # 重叠方式：已完成 OVERLAP_MS + 截图识别，只需按压剩余时间
                    actual_ms, jitter_factor = apply_jitter(press_ms)
                    elapsed_ms = (time.perf_counter() - t_press_start) * 1000.0
                    remaining_ms = max(0.0, actual_ms - elapsed_ms)
                    if remaining_ms > 0:
                        time.sleep(remaining_ms / 1000.0)
                    self.mouse.release(Button.left)
                    time.sleep(settle_ms / 1000.0)
                    time.sleep(0.05)
        print("已退出。")

def cmd_run():
    region = detect_region()
    runner = Runner(region)
    listener = keyboard.Listener(on_press=runner.on_key)
    listener.start()
    try:
        runner.loop()
    finally:
        listener.stop()

def main():
    set_dpi_aware()
    ap = argparse.ArgumentParser(description="跳一跳 电脑端自动脚本（图像识别）")
    ap.add_argument("cmd", nargs="?", default="run",
                    choices=["test", "run"],
                    help="run=自动跳（默认）  test=测识别")
    args = ap.parse_args()
    if args.cmd == "test":
        cmd_test()
    else:
        cmd_run()

if __name__ == "__main__":
    main()
