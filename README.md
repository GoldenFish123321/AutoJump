# 跳一跳 电脑端自动脚本

基于图像识别 + 物理模型的微信跳一跳 PC 端自动脚本。通过 Windows API 自动定位游戏窗口，正交相机公式精确计算跳跃参数。

## 原理

### 识别算法

1. 颜色掩码定位棋子底部中心（默认蓝灰皮肤，可配置 RGB 区间）
2. 从上往下逐行扫描第一块非背景像素，得到下一目标块的顶点 x（已排除棋子掩码区域）
3. 等距投影几何（tan30°）推出目标中心，算出像素距离 D
4. 在几何中心附近搜索 `F5F5F5` 完美跳跃白点：命中则用 `cv2.fitEllipse` 椭圆拟合精确校准目标中心，并据此判定上一跳是否完美

### 物理模型（根据游戏源码反推）

#### 符号

| 符号 | 含义 | 单位 |
|------|------|------|
| $h$ | 按压时间 | 秒 (s) |
| $D$ | 屏幕上识别到的跳跃距离 | 像素 (px) |
| $p$ | 实际按压时长 | 毫秒 (ms) |
| $v_z(h)$ | 游戏内水平初速度 | 游戏单位/s |
| $v_y(h)$ | 游戏内竖直初速度 | 游戏单位/s |
| $g$ | 游戏内重力加速度 | 游戏单位/s² |
| $t_{\text{air}}(h)$ | 空中理论飞行时间 | 秒 (s) |
| $x(h)$ | 游戏内 3D 水平移动距离 | 游戏单位 |
| $k$ | 屏幕像素与 3D 距离的比例系数 | px/游戏单位 |
| $W$ | 游戏窗口短边像素（自动获取） | px |
| $T_{\text{stop}}(h)$ | 停稳时间（含飞行+落地动画+场景移动） | 毫秒 (ms) |

#### 源码常量

$$v_{z,\text{inc}} = 70,\quad v_{y,0} = 135,\quad v_{y,\text{inc}} = 15,\quad g = 720$$

#### 速度公式（含上限）

$$v_z(h) = \min(70h,\ 150)$$

$$v_y(h) = \min(135 + 15h,\ 180)$$

#### 空中飞行时间

$$t_{\text{air}}(h) = \frac{2v_y(h)}{g}$$

#### 游戏内 3D 水平距离

$$x(h) = v_z(h) \cdot t_{\text{air}}(h)$$

#### 屏幕像素与 3D 距离的关系 — 正交相机精确公式

由游戏源码正交相机参数精确推导（无需手动拟合）：

$$k = \sqrt{\frac{2}{3}} \cdot W \cdot \frac{736}{414 \cdot 60} \approx 0.02418 \cdot W$$

其中 $W = \min(\text{region\_w},\ \text{region\_h}) - \text{WINDOW\_UI\_OFFSET}$，即游戏窗口短边像素。

$$D = k \cdot x(h)$$

#### 按压时间反解

由于 $x(h)$ 含 $\min$ 上限不可直接求解析解，使用**二分法**（50 次迭代，精度 > $10^{-15}$）从 $D$ 反解 $h$，对所有距离均精确：

$$h = \text{bisect}\left(x(h) = \frac{D}{k}\right)$$

$$p = 1000h\ \text{ms}$$

#### 停稳时间模型（全部参数来自源码精确值）

着陆后以下动画**并行**执行，取最长者：

| 动画 | 时长 | 来源 |
|------|------|------|
| 挤压动画 | 300ms | `customAnimation.to body.scale` |
| 方块回弹 | 500ms | `block.rebound()` Elastic |
| 粒子飞散 | 300~500ms | `scatterParticles()` |
| 得分显示 | 700ms | `showAddScore` TweenAnimation |
| 完美光环 | 480~900ms | 30fps 录像实测: $\min(900,\ 340+140\cdot\text{combo}//2)$ ms |
| 场景移动 | $50 \cdot x_{\text{curr}}$ ms | `moveGradually()` 源码 $500\cdot |vars|/10$ |

$$T_{\text{stop}} = t_{\text{air,ms}} + \max(700,\ 50 \cdot x_{\text{curr}},\ \text{halo\_ms}) + 100\ \text{ms}$$

其中 $100\text{ms}$ 为安全余量。

#### 完美光环时长

根据 combo 倍率线性拟合（30fps 录像实测）：

| Combo | 光环时长 |
|-------|----------|
| 2x | ~480ms |
| 4x | ~620ms |
| 6x | ~760ms |
| 8x+ | ~900ms（封顶） |

### 按压抖动

对按压时间施加三角分布对称抖动（幅度由 `PHYS_JITTER` 控制，默认 0 即关闭），避免被检测为脚本：

$$p_{\text{实际}} = p_{\text{名义}} \times f,\quad f \sim \text{Tri}(1-r,\ 1+r,\ 1.0)$$

### 重叠优化

按压与截图识别重叠执行：先按压一段（`OVERLAP_MS`，默认 0 即关闭），期间完成截图+识别，再将剩余按压时间补足。因为游戏最小按压阈值 > 160ms，首段按压 < 160ms 不会触发跳跃。


## 用法

### 环境

```bash
pip install -r requirements.txt
```

### 第一步：自动识别游戏区域

脚本通过 Windows API 自动查找标题包含「跳一跳」的窗口，获取**客户区**（`GetClientRect` + `ClientToScreen`，已自动排除标题栏和边框）作为游戏区域。

```bash
python jump_pc.py region
```

识别结果存入 `config.json`，匹配预览图存到 `debug/region_match.png`（黄框标注）。

若自动识别失败（窗口标题不含"跳一跳"），会回退到**手动框选**模式。也可修改脚本顶部 `WINDOW_TITLE` 常量匹配其他窗口标题。若模拟器内部有边栏，可设置 `WINDOW_UI_OFFSET` 扣除。

### 第二步：测试识别

```bash
python jump_pc.py test
```

截一帧看识别对不对，标注图存到 `debug/test.png`：
- 🟢 绿点 = 棋子落脚点
- 🔴 红点 = 目标中心
- 🟣 品红 = 完美白点（F5F5F5）

### 第三步：自动跳

```bash
python jump_pc.py run
```

首次运行会自动识别区域。脚本会自动将游戏窗口拉到前台。

## 运行中热键

| 按键 | 功能 |
|------|------|
| `空格` | 暂停 / 继续 |
| `d` | 存一张当前识别 debug 图（`debug/frame_XXXX.png`） |
| `q` / `Esc` | 退出 |

## 文件结构

```
tyt/
├── jump_pc.py              # 主脚本（识别 + 物理模型 + 自动跳跃）
├── config.json             # 配置（游戏区域、按压系数）
├── requirements.txt        # Python 依赖
├── README.md
├── .gitignore
└── debug/                  # 调试输出（自动创建）
    ├── region_match.png    # 区域匹配预览（黄框标注）
    ├── test.png            # 识别测试标注图
    ├── frame_XXXX.png      # 手动存图（按 d）
    └── non_perfect_XXXX.png # 非完美跳自动存图
```

## 依赖

| 包 | 用途 |
|----|------|
| Python 3.x | 运行环境 |
| opencv-python | 图像识别、轮廓检测、椭圆拟合 |
| mss | 屏幕区域截图 |
| pynput | 鼠标长按控制 + 全局键盘监听 |
| numpy | 数组运算 |

## 配置

`config.json` 示例：

```json
{
  "region": [1035, 235, 490, 914],
  "press_coefficient": 2.0
}
```

| 字段 | 说明 |
|------|------|
| `region` | 游戏画面在屏幕上的矩形区域 `[左, 上, 宽, 高]`（物理像素） |
| `press_coefficient` | 按压时间倍率微调（默认 2.0，一般无需修改） |

### 可调参数（脚本顶部常量）

| 常量 | 默认值 | 说明 |
|------|--------|------|
| `WINDOW_TITLE` | `"跳一跳"` | 匹配的窗口标题关键字 |
| `WINDOW_UI_OFFSET` | `0` | 模拟器内部边栏扣除（像素） |
| `PIECE_R/G/B` | `(50,60)/(53,63)/(95,110)` | 棋子颜色 RGB 区间（默认蓝灰皮肤） |
| `PHYS_JITTER` | `0` | 按压抖动幅度（0=关闭，建议 ≤0.02） |
| `OVERLAP_MS` | `0` | 按压截图重叠优化时长（0=关闭，建议 ≤100） |
| `PHYS_SETTLE_SAFETY_MS` | `100` | 停稳安全余量（毫秒） |

## License

MIT
| `phys_k` | 屏幕像素与 3D 距离的比例系数，自动校准 |
