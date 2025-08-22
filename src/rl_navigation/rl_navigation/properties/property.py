# !/usr/bin/env python3
# property.py — forward-only blocking using FRONT-SECTOR distance
# - Only blocks near-straight "forward" (|wz| small), not real turns.
# - Uses /scan_front_min (preferred) or falls back to /scan_min.
# - Hysteresis near threshold to avoid chattering.
# - Cooldown by COMMAND COUNT (robust to sim acceleration).

import json, math, time  # time 仅用于时间戳记录；冷却不再依赖“秒”

# ===== Tunables (monitor-only) =====
STOP_DIST       = 0.21    # m: 阈值（配合滞回）
HYST            = 0.02    # m: 滞回带；释放需要 > STOP_DIST + 2*HYST
VX_FWD_THRESH   = 0.20    # m/s: 认为“前进”的线速度阈值
FWD_WZ_MAX      = 0.20    # rad/s: 仅当 |wz| ≤ 此值认为是“近似直行”（避免误伤转向）
COOLDOWN_CMDS   = 3       # [NEW] 被拦一次后，接下来屏蔽这么多条“近似直行”命令

# ===== Predicate interface =====
predicates = dict(is_cmd=False, forward=False, too_close=False, safe=True)

# Velocity topics that may carry Twist-like commands
_CMD_TOPICS = {"/cmd_vel_raw", "/cmd_vel_raw_mon", "/cmd_vel"}

# Debug log (one JSON line per processed message)
_LOG_PATH = "property_debug.jsonl"

DEBUG_LOG = True          # 关闭则不写任何调试行
_LOG_PATH = "property_debug.jsonl"  # 保留文件名以便随时开启

def _dbg(payload: dict):
    if not DEBUG_LOG:
        return  # 彻底静默
    try:
        with open(_LOG_PATH, "a") as f:
            f.write(json.dumps(payload, ensure_ascii=False, default=str) + "\n")
    except Exception:
        pass

def _f(x):
    try: return float(x)
    except Exception: return None

def _vx_wz_from(data):
    vx = wz = None
    if isinstance(data, dict):
        lin = data.get("linear"); ang = data.get("angular")
        if isinstance(lin, dict): vx = _f(lin.get("x"))
        if isinstance(ang, dict): wz = _f(ang.get("z"))
        if vx is None and "linear_x" in data:  vx = _f(data["linear_x"])
        if wz is None and "angular_z" in data: wz = _f(data["angular_z"])
        if vx is None and isinstance(lin, (list, tuple)) and len(lin) > 0: vx = _f(lin[0])
        if wz is None and isinstance(ang, (list, tuple)) and len(ang) > 2: wz = _f(ang[2])
    return vx, wz

def _looks_like_twist(data):
    if not isinstance(data, dict): return False
    if "linear" in data and "angular" in data: return True
    if "linear_x" in data or "angular_z" in data: return True
    if isinstance(data.get("linear"), (list, tuple)) or isinstance(data.get("angular"), (list, tuple)): return True
    return False

# ---------------------- State ----------------------
_last_front_min   = None     # /scan_front_min（首选）
_last_global_min  = None     # /scan_min（回退）
_block_left       = 0        # [NEW] 冷却剩余需屏蔽的“近似直行”命令数

def abstract_message(message: dict) -> dict:
    global _last_front_min, _last_global_min, _block_left

    now   = message.get("timestamp") or message.get("time") or time.time()
    topic = message.get("topic", "")
    data  = message.get("data", {})

    # 容错：某些桥会把字段放在顶层
    if not isinstance(data, dict) or len(data) == 0:
        candidates = {}
        for k in ("linear","angular","linear_x","angular_z","twist","msg","data"):
            if k in message: candidates[k] = message[k]
        if "msg" in candidates and isinstance(candidates["msg"], str):
            import json as _json
            try: candidates.update(_json.loads(candidates["msg"]))
            except Exception: pass
        if candidates: data = candidates

    # --- Sensor updates ---
    scan_update = False
    if topic == "/scan_front_min":
        v = _f(data) if not isinstance(data, dict) else _f(data.get("data", data))
        if v is not None and math.isfinite(v):
            _last_front_min = v
            scan_update = True
    elif topic == "/scan_min":
        v = _f(data) if not isinstance(data, dict) else _f(data.get("data", data))
        if v is not None and math.isfinite(v):
            _last_global_min = v
            if _last_front_min is None:
                _last_front_min = v  # 无 front_min 时临时用全局最小
            scan_update = True

    # --- Command parsing ---
    is_cmd = (topic in _CMD_TOPICS) or _looks_like_twist(data)
    vx, wz = _vx_wz_from(data) if is_cmd else (None, None)

    # [CHANGED] 仅将“近似直行”视作 forward：vx 足够且 |wz| 小
    forward = (vx is not None and vx > VX_FWD_THRESH and (wz is None or abs(wz) <= FWD_WZ_MAX))

    # --- Distance & hysteresis ---
    dist = _last_front_min if _last_front_min is not None else _last_global_min
    prev_safe = predicates.get("safe", True)

    if dist is None:
        too_close = False  # 无感知时不过度保守；按需可改成 True
    else:
        if prev_safe:
            too_close = (dist < (STOP_DIST + HYST))
        else:
            too_close = (dist < (STOP_DIST + 2.0 * HYST))

    # --- Cooldown by command count (robust to sim acceleration) ---
    # [NEW] 若处于冷却期，则继续屏蔽“近似直行”，并在收到一条命令后递减
    in_cmd_cooldown = (_block_left > 0)
    if is_cmd and in_cmd_cooldown:
        _block_left -= 1  # 每来一条命令就减少一次

    # 触发阻断：forward 且 (too_close 或 冷却期)
    block_forward = is_cmd and forward and (too_close or in_cmd_cooldown)

    # 若本次真正触发了“过近阻断”，启动/刷新命令冷却
    if is_cmd and forward and too_close:
        _block_left = max(_block_left, COOLDOWN_CMDS)  # [NEW]

    safe = not block_forward

    # --- Debug log line ---
    _dbg({
        "t": float(now),
        "topic": topic,
        "is_cmd": bool(is_cmd),
        "vx": vx, "wz": wz,
        "forward": bool(forward),
        "front_min": _last_front_min,
        "global_min": _last_global_min,
        "scan_update": scan_update,
        "too_close": bool(too_close),
        "cooldown_left": _block_left,      # [NEW]
        "safe": bool(safe),
        "params": dict(
            STOP_DIST=STOP_DIST, HYST=HYST,
            VX_FWD_THRESH=VX_FWD_THRESH, FWD_WZ_MAX=FWD_WZ_MAX,
            COOLDOWN_CMDS=COOLDOWN_CMDS
        )
    })

    predicates["is_cmd"]    = bool(is_cmd)
    predicates["forward"]   = bool(forward)
    predicates["too_close"] = bool(too_close)
    predicates["safe"]      = bool(safe)
    return predicates


# The temporal logic property (keep simple)
PROPERTY = "{ safe }"