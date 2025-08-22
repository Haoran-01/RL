# property.py — 距离 < 0.25m 禁止直行（带详细 DEBUG 日志）
import json, time
STOP_DIST = 0.25  # m

# 返回给监控器的谓词
predicates = dict(is_cmd=False, forward=False, too_close=False, safe=True)

# 把这些话题视作“速度指令”（入口/出口/最终全兼容）
_CMD_TOPICS = {"/cmd_vel_raw", "/cmd_vel_raw_mon", "/cmd_vel"}

# 调试：逐条事件落盘（JSONL）
_LOG_PATH = "property_debug.jsonl"
def _dbg(payload: dict):
    try:
        with open(_LOG_PATH, "a") as f:
            f.write(json.dumps(payload, ensure_ascii=False, default=str) + "\n")
    except Exception:
        pass

# 缓存最近的 scan 值
_last_scan = None

def _f(x):
    try:
        return float(x)
    except Exception:
        return None

def _vx_wz_from(data):
    """尽量从各种形状里提取 vx/wz."""
    vx = wz = None
    if isinstance(data, dict):
        lin = data.get("linear"); ang = data.get("angular")
        if isinstance(lin, dict): vx = _f(lin.get("x"))
        if isinstance(ang, dict): wz = _f(ang.get("z"))
        if vx is None and "linear_x" in data: vx = _f(data["linear_x"])
        if wz is None and "angular_z" in data: wz = _f(data["angular_z"])
        if vx is None and isinstance(lin, (list, tuple)) and len(lin) > 0: vx = _f(lin[0])
        if wz is None and isinstance(ang, (list, tuple)) and len(ang) > 2: wz = _f(ang[2])
    return vx, wz

def _looks_like_twist(data):
    if not isinstance(data, dict): return False
    if "linear" in data and "angular" in data: return True
    if "linear_x" in data or "angular_z" in data: return True
    if isinstance(data.get("linear"), (list, tuple)) or isinstance(data.get("angular"), (list, tuple)):
        return True
    return False

def abstract_message(message: dict) -> dict:
    global _last_scan

    now   = message.get("timestamp") or message.get("time") or time.time()
    topic = message.get("topic", "")
    data  = message.get("data", {})

    raw = message

    # 如果 data 是空的，尝试从顶层提取可能的 Twist 字段
    if not isinstance(data, dict) or len(data) == 0:
        candidates = {}
        for k in ("linear", "angular", "linear_x", "angular_z", "twist", "msg"):
            if k in raw:
                candidates[k] = raw[k]
        # 某些实现把完整消息放在 'msg'（或字符串 JSON）
        if "msg" in candidates and isinstance(candidates["msg"], str):
            import json as _json
            try:
                candidates.update(_json.loads(candidates["msg"]))
            except Exception:
                pass
        if candidates:
            data = candidates


    # 记录原始数据的“形状”，避免把超大 data 整体写入文件
    shape = dict(
        type=str(type(data).__name__),
        keys=list(data.keys()) if isinstance(data, dict) else None,
        preview=str(data)[:200]
    )

    # 1) 更新最近的距离（/scan_min: Float32 或 {"data": v}）
    scan_update = False
    if topic == "/scan_min":
        v = _f(data) if not isinstance(data, dict) else _f(data.get("data", data))
        if v is not None:
            _last_scan = v
            scan_update = True

    # 2) 识别“是否是命令事件” + 取 vx/wz
    is_cmd = (topic in _CMD_TOPICS) or _looks_like_twist(data)
    vx, wz = _vx_wz_from(data) if is_cmd else (None, None)
    forward = (vx is not None and vx > 0.15)

    # 3) 判定：仅当“命令事件 且 向前 且 距离小于阈值”不安全
    too_close = (_last_scan is not None and _last_scan < STOP_DIST)
    safe = (not is_cmd) or (not (forward and too_close))

    # 4) 写调试日志（每条事件一行）
    _dbg({
        "t": float(now),
        "topic": topic,
        "shape": shape,
        "is_cmd": bool(is_cmd),
        "vx": vx, "wz": wz,
        "forward": bool(forward),
        "last_scan": _last_scan,
        "scan_update": scan_update,
        "too_close": bool(too_close),
        "safe": bool(safe),
    })

    # 5) 返回谓词
    predicates["is_cmd"]    = bool(is_cmd)
    predicates["forward"]   = bool(forward)
    predicates["too_close"] = bool(too_close)
    predicates["safe"]      = bool(safe)
    
    return predicates

PROPERTY ="{ safe }"




# # property.py —— 只通过监控来影响训练（不改RL）
# # 1) 距离阈值 + 迟滞 + 冷却：贴近障碍禁止“向前”
# # 2) 旋转检测：长时间大角速度 + 前方无改善 → 触发一次制动打断“绕圈”
# import json, time, collections

# # —— 可调监控参数（只改这里，不碰RL代码）——
# STOP_DIST    = 0.25      # 与障碍的“刹停阈值”（米）
# CLEAR_MARGIN = 0.07      # 迟滞：恢复前进需要更大的安全余量
# COOLDOWN_S   = 0.5       # 被阻断后这段时间内继续禁止“向前”，避免抖动
# SPIN_WZ      = 0.8       # 认为在“原地打转”的 |wz| 阈值（rad/s）
# SPIN_WIN_S   = 1.0       # 持续这么久都在大角速度才算
# FRONT_IMPROVE= 0.03      # 这段时间前方距离无明显改善，视为无效旋转
# SPIN_COOLDOWN= 0.8       # 触发一次“反旋转制动”后冷却，避免连续触发

# # —— 输出谓词（与框架约定）——
# predicates = dict(is_cmd=False, forward=False, too_close=False, safe=True)

# _CMD_TOPICS = {"/cmd_vel_raw", "/cmd_vel_raw_mon", "/cmd_vel"}
# _LOG_PATH   = "property_debug.jsonl"

# _last_scan   = None
# _last_block_t= 0.0
# _spin_buf    = collections.deque(maxlen=100)  # (t, |wz|, scan)
# _last_spin_t = 0.0

# def _dbg(payload: dict):
#     try:
#         with open(_LOG_PATH, "a") as f:
#             f.write(json.dumps(payload, ensure_ascii=False, default=str) + "\n")
#     except Exception:
#         pass

# def _f(x):
#     try: return float(x)
#     except Exception: return None

# def _vx_wz_from(data):
#     vx = wz = None
#     if isinstance(data, dict):
#         lin = data.get("linear"); ang = data.get("angular")
#         if isinstance(lin, dict): vx = _f(lin.get("x"))
#         if isinstance(ang, dict): wz = _f(ang.get("z"))
#         if vx is None and "linear_x" in data:  vx = _f(data["linear_x"])
#         if wz is None and "angular_z" in data: wz = _f(data["angular_z"])
#         if vx is None and isinstance(lin, (list, tuple)) and len(lin) > 0: vx = _f(lin[0])
#         if wz is None and isinstance(ang, (list, tuple)) and len(ang) > 2: wz = _f(ang[2])
#     return vx, wz

# def _looks_like_twist(data):
#     if not isinstance(data, dict): return False
#     if "linear" in data and "angular" in data: return True
#     if "linear_x" in data or "angular_z" in data: return True
#     if isinstance(data.get("linear"), (list, tuple)) or isinstance(data.get("angular"), (list, tuple)):
#         return True
#     return False

# def abstract_message(message: dict) -> dict:
#     global _last_scan, _last_block_t, _last_spin_t

#     now   = message.get("timestamp") or message.get("time") or time.time()
#     topic = message.get("topic", "")
#     data  = message.get("data", {})

#     raw = message
#     if not isinstance(data, dict) or len(data) == 0:
#         # 兼容不同打包格式
#         candidates = {}
#         for k in ("linear", "angular", "linear_x", "angular_z", "twist", "msg"):
#             if k in raw: candidates[k] = raw[k]
#         if "msg" in candidates and isinstance(candidates["msg"], str):
#             import json as _json
#             try: candidates.update(_json.loads(candidates["msg"]))
#             except Exception: pass
#         if candidates: data = candidates

#     # —— 更新最近一次 /scan_min —— 
#     scan_update = False
#     if topic == "/scan_min":
#         v = _f(data) if not isinstance(data, dict) else _f(data.get("data", data))
#         if v is not None:
#             _last_scan = v
#             scan_update = True

#     # —— 是否速度指令 + 提取 vx/wz ——
#     is_cmd = (topic in _CMD_TOPICS) or _looks_like_twist(data)
#     vx, wz = _vx_wz_from(data) if is_cmd else (None, None)
#     forward = (vx is not None and vx > 0.15)

#     # —— 规则1：贴近直冲禁止（迟滞 + 冷却）——
#     too_close = (_last_scan is not None and _last_scan < STOP_DIST)
#     in_cooldown = (now - _last_block_t) < COOLDOWN_S
#     # “允许重新前进”的条件：距离 >= STOP_DIST + CLEAR_MARGIN 且 冷却结束
#     forward_reenable = (_last_scan is not None and _last_scan >= (STOP_DIST + CLEAR_MARGIN) and not in_cooldown)

#     block_forward = is_cmd and forward and (too_close or not forward_reenable)
#     if block_forward:
#         _last_block_t = now  # 记录阻断时间，进入冷却

#     # —— 规则2：原地打转检测（打断“绕圈”）——
#     # 条件：|wz| 长时间较大 + 这段时间内前方距离无改善
#     spin_block = False
#     if is_cmd and wz is not None and _last_scan is not None:
#         _spin_buf.append((float(now), abs(wz), float(_last_scan)))
#         # 滑窗内统计
#         t0 = now - SPIN_WIN_S
#         buf = [x for x in _spin_buf if x[0] >= t0]
#         if len(buf) >= 3:
#             wz_ok   = all(w >= SPIN_WZ for (_, w, __) in buf)
#             impr    = (buf[-1][2] - buf[0][2])  # scan 变大=更通畅
#             cool_ok = (now - _last_spin_t) >= SPIN_COOLDOWN
#             # 仅在“非向前”或“向前无意义”时触发（不去改变转向策略，只打断）
#             if wz_ok and impr < FRONT_IMPROVE and cool_ok:
#                 spin_block   = True
#                 _last_spin_t = now

#     safe = not (block_forward or spin_block)

#     # —— 调试日志 —— 
#     _dbg({
#         "t": float(now), "topic": topic,
#         "is_cmd": bool(is_cmd),
#         "vx": vx, "wz": wz,
#         "forward": bool(forward),
#         "scan": _last_scan, "scan_update": scan_update,
#         "too_close": bool(too_close),
#         "in_cooldown": bool(in_cooldown),
#         "spin_block": bool(spin_block),
#         "safe": bool(safe)
#     })

#     # —— 返回谓词 —— 
#     predicates["is_cmd"]    = bool(is_cmd)
#     predicates["forward"]   = bool(forward)
#     predicates["too_close"] = bool(too_close)
#     predicates["safe"]      = bool(safe)
#     return predicates

# PROPERTY = "{ safe }"

# # !/usr/bin/env python3
# property.py — forward-only blocking using FRONT-SECTOR distance
# - Only blocks near-straight "forward" (|wz| small), not real turns.
# - Uses /scan_front_min (preferred) or falls back to /scan_min.
# - Hysteresis near threshold to avoid chattering.
# - Cooldown by COMMAND COUNT (robust to sim acceleration).

# import json, math, time  # time 仅用于时间戳记录；冷却不再依赖“秒”

# # ===== Tunables (monitor-only) =====
# STOP_DIST       = 0.21    # m: 阈值（配合滞回）
# HYST            = 0.02    # m: 滞回带；释放需要 > STOP_DIST + 2*HYST
# VX_FWD_THRESH   = 0.20    # m/s: 认为“前进”的线速度阈值
# FWD_WZ_MAX      = 0.20    # rad/s: 仅当 |wz| ≤ 此值认为是“近似直行”（避免误伤转向）
# COOLDOWN_CMDS   = 3       # [NEW] 被拦一次后，接下来屏蔽这么多条“近似直行”命令

# # ===== Predicate interface =====
# predicates = dict(is_cmd=False, forward=False, too_close=False, safe=True)

# # Velocity topics that may carry Twist-like commands
# _CMD_TOPICS = {"/cmd_vel_raw", "/cmd_vel_raw_mon", "/cmd_vel"}

# # Debug log (one JSON line per processed message)
# _LOG_PATH = "property_debug.jsonl"

# DEBUG_LOG = True          # 关闭则不写任何调试行
# _LOG_PATH = "property_debug.jsonl"  # 保留文件名以便随时开启

# def _dbg(payload: dict):
#     if not DEBUG_LOG:
#         return  # 彻底静默
#     try:
#         with open(_LOG_PATH, "a") as f:
#             f.write(json.dumps(payload, ensure_ascii=False, default=str) + "\n")
#     except Exception:
#         pass

# def _f(x):
#     try: return float(x)
#     except Exception: return None

# def _vx_wz_from(data):
#     vx = wz = None
#     if isinstance(data, dict):
#         lin = data.get("linear"); ang = data.get("angular")
#         if isinstance(lin, dict): vx = _f(lin.get("x"))
#         if isinstance(ang, dict): wz = _f(ang.get("z"))
#         if vx is None and "linear_x" in data:  vx = _f(data["linear_x"])
#         if wz is None and "angular_z" in data: wz = _f(data["angular_z"])
#         if vx is None and isinstance(lin, (list, tuple)) and len(lin) > 0: vx = _f(lin[0])
#         if wz is None and isinstance(ang, (list, tuple)) and len(ang) > 2: wz = _f(ang[2])
#     return vx, wz

# def _looks_like_twist(data):
#     if not isinstance(data, dict): return False
#     if "linear" in data and "angular" in data: return True
#     if "linear_x" in data or "angular_z" in data: return True
#     if isinstance(data.get("linear"), (list, tuple)) or isinstance(data.get("angular"), (list, tuple)): return True
#     return False

# # ---------------------- State ----------------------
# _last_front_min   = None     # /scan_front_min（首选）
# _last_global_min  = None     # /scan_min（回退）
# _block_left       = 0        # [NEW] 冷却剩余需屏蔽的“近似直行”命令数

# def abstract_message(message: dict) -> dict:
#     global _last_front_min, _last_global_min, _block_left

#     now   = message.get("timestamp") or message.get("time") or time.time()
#     topic = message.get("topic", "")
#     data  = message.get("data", {})

#     # 容错：某些桥会把字段放在顶层
#     if not isinstance(data, dict) or len(data) == 0:
#         candidates = {}
#         for k in ("linear","angular","linear_x","angular_z","twist","msg","data"):
#             if k in message: candidates[k] = message[k]
#         if "msg" in candidates and isinstance(candidates["msg"], str):
#             import json as _json
#             try: candidates.update(_json.loads(candidates["msg"]))
#             except Exception: pass
#         if candidates: data = candidates

#     # --- Sensor updates ---
#     scan_update = False
#     if topic == "/scan_front_min":
#         v = _f(data) if not isinstance(data, dict) else _f(data.get("data", data))
#         if v is not None and math.isfinite(v):
#             _last_front_min = v
#             scan_update = True
#     elif topic == "/scan_min":
#         v = _f(data) if not isinstance(data, dict) else _f(data.get("data", data))
#         if v is not None and math.isfinite(v):
#             _last_global_min = v
#             if _last_front_min is None:
#                 _last_front_min = v  # 无 front_min 时临时用全局最小
#             scan_update = True

#     # --- Command parsing ---
#     is_cmd = (topic in _CMD_TOPICS) or _looks_like_twist(data)
#     vx, wz = _vx_wz_from(data) if is_cmd else (None, None)

#     # [CHANGED] 仅将“近似直行”视作 forward：vx 足够且 |wz| 小
#     forward = (vx is not None and vx > VX_FWD_THRESH and (wz is None or abs(wz) <= FWD_WZ_MAX))

#     # --- Distance & hysteresis ---
#     dist = _last_front_min if _last_front_min is not None else _last_global_min
#     prev_safe = predicates.get("safe", True)

#     if dist is None:
#         too_close = False  # 无感知时不过度保守；按需可改成 True
#     else:
#         if prev_safe:
#             too_close = (dist < (STOP_DIST + HYST))
#         else:
#             too_close = (dist < (STOP_DIST + 2.0 * HYST))

#     # --- Cooldown by command count (robust to sim acceleration) ---
#     # [NEW] 若处于冷却期，则继续屏蔽“近似直行”，并在收到一条命令后递减
#     in_cmd_cooldown = (_block_left > 0)
#     if is_cmd and in_cmd_cooldown:
#         _block_left -= 1  # 每来一条命令就减少一次

#     # 触发阻断：forward 且 (too_close 或 冷却期)
#     block_forward = is_cmd and forward and (too_close or in_cmd_cooldown)

#     # 若本次真正触发了“过近阻断”，启动/刷新命令冷却
#     if is_cmd and forward and too_close:
#         _block_left = max(_block_left, COOLDOWN_CMDS)  # [NEW]

#     safe = not block_forward

#     # --- Debug log line ---
#     _dbg({
#         "t": float(now),
#         "topic": topic,
#         "is_cmd": bool(is_cmd),
#         "vx": vx, "wz": wz,
#         "forward": bool(forward),
#         "front_min": _last_front_min,
#         "global_min": _last_global_min,
#         "scan_update": scan_update,
#         "too_close": bool(too_close),
#         "cooldown_left": _block_left,      # [NEW]
#         "safe": bool(safe),
#         "params": dict(
#             STOP_DIST=STOP_DIST, HYST=HYST,
#             VX_FWD_THRESH=VX_FWD_THRESH, FWD_WZ_MAX=FWD_WZ_MAX,
#             COOLDOWN_CMDS=COOLDOWN_CMDS
#         )
#     })

#     predicates["is_cmd"]    = bool(is_cmd)
#     predicates["forward"]   = bool(forward)
#     predicates["too_close"] = bool(too_close)
#     predicates["safe"]      = bool(safe)
#     return predicates


# # The temporal logic property (keep simple)
# PROPERTY = "{ safe }"



# # property.py — pass-through (never filter anything)

# # 可选：保留这些键便于调试/一致性
# predicates = dict(is_cmd=False, forward=False, too_close=False, safe=True)

# #（可选）声明哪些话题算“指令”，仅用于观测，不影响结果
# _CMD_TOPICS = {"/cmd_vel_raw", "/cmd_vel_raw_mon", "/cmd_vel"}

# def abstract_message(message: dict) -> dict:
#     topic = message.get("topic", "")
#     data  = message.get("data", {})

#     # 仅用于填充观测字段，方便你 echo 看状态；不影响 safe 的结果
#     predicates["is_cmd"]   = topic in _CMD_TOPICS
#     predicates["forward"]  = False
#     predicates["too_close"]= False

#     # 关键：始终判定安全
#     predicates["safe"] = True
#     return predicates

# # 性质：历史上一直安全（因为上面总是 True，自然不会触发过滤）
# PROPERTY = r'historically{ safe }'


