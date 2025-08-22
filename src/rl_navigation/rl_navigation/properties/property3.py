# property.py — stop forward if too close (TTL + latch + hysteresis + failsafe)

# import time

# # --- 超参数（可按需微调） ---
# ENTER_DIST = 0.25          # 进入危险：< 0.25 m
# EXIT_DIST  = 0.30          # 解除危险：> 0.30 m（迟滞，避免抖动）
# SCAN_TTL   = 0.5           # s：最近 0.5s 的 scan 才有效
# LATCH_HOLD = 0.4           # s：进危险后至少保持拦截 0.4s
# FAILSAFE_IF_NO_RECENT = True  # 无近期 scan 时禁止向前（更安全）

# # 兼容的速度指令话题名
# _CMD_TOPICS = {"/cmd_vel_raw", "/cmd_vel_raw_mon", "/cmd_vel"}

# # 内部状态
# _last_scan = None
# _last_scan_time = None
# _latch_until = 0.0

# # 对外谓词
# predicates = dict(is_cmd=False, forward=False, too_close=False, safe=True)

# def _now(msg):
#     t = msg.get("timestamp") or msg.get("time")
#     return float(t) if isinstance(t, (int, float)) else time.time()

# def _f(x):
#     try:
#         return float(x)
#     except Exception:
#         return None

# def _vx_from(data):
#     if not isinstance(data, dict):
#         return None
#     # 标准 Twist
#     try:
#         return float(data["linear"]["x"])
#     except Exception:
#         pass
#     # 扁平/数组兜底
#     if "linear_x" in data:
#         return _f(data["linear_x"])
#     lin = data.get("linear")
#     if isinstance(lin, (list, tuple)) and len(lin) > 0:
#         return _f(lin[0])
#     return None

# def abstract_message(message: dict) -> dict:
#     global _last_scan, _last_scan_time, _latch_until

#     topic = message.get("topic", "")
#     data  = message.get("data", {})
#     now   = _now(message)

#     # 1) 更新 scan 状态（支持 Float32 或 {"data": v}）
#     if topic == "/scan_min":
#         v = _f(data) if not isinstance(data, dict) else _f(data.get("data", data))
#         if v is not None:
#             _last_scan = v
#             _last_scan_time = now
#             # 迟滞 + 粘滞保持
#             if v < ENTER_DIST:
#                 _latch_until = max(_latch_until, now + LATCH_HOLD)
#             elif v > EXIT_DIST and now >= _latch_until:
#                 _latch_until = 0.0

#     # 2) 是否是速度指令 + 是否向前
#     is_cmd = topic in _CMD_TOPICS
#     forward = False
#     if is_cmd:
#         vx = _vx_from(data)
#         forward = (vx is not None and vx > 0.0)

#     # 3) 近距/新鲜度/失效保护
#     scan_fresh = (_last_scan_time is not None) and ((now - _last_scan_time) <= SCAN_TTL)
#     too_close  = (scan_fresh and (_last_scan is not None) and (_last_scan < ENTER_DIST))
#     latched    = (now < _latch_until)
#     no_recent  = (not scan_fresh)

#     # “不安全”条件：向前 且（太近 或 处于保持窗口 或（无近期scan且启用失效保护））
#     unsafe = forward and (too_close or latched or (FAILSAFE_IF_NO_RECENT and no_recent))

#     # 4) 仅当“速度事件且不安全”时置非安全；其它事件不改变安全性
#     safe = (not is_cmd) or (not unsafe)

#     predicates["is_cmd"] = bool(is_cmd)
#     predicates["forward"] = bool(forward)
#     predicates["too_close"] = bool(too_close or latched)
#     predicates["safe"] = bool(safe)
#     return predicates

# 性质：到目前为止一直安全
# PROPERTY = r'historically{ safe: true }'


# property.py — pass-through (never filter anything)

# 可选：保留这些键便于调试/一致性
predicates = dict(is_cmd=False, forward=False, too_close=False, safe=True)

#（可选）声明哪些话题算“指令”，仅用于观测，不影响结果
_CMD_TOPICS = {"/cmd_vel_raw", "/cmd_vel_raw_mon", "/cmd_vel"}

def abstract_message(message: dict) -> dict:
    topic = message.get("topic", "")
    data  = message.get("data", {})

    # 仅用于填充观测字段，方便你 echo 看状态；不影响 safe 的结果
    predicates["is_cmd"]   = topic in _CMD_TOPICS
    predicates["forward"]  = False
    predicates["too_close"]= False

    # 关键：始终判定安全
    predicates["safe"] = True
    return predicates

# 性质：历史上一直安全（因为上面总是 True，自然不会触发过滤）
PROPERTY = r'historically{ safe }'

