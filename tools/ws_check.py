#!/usr/bin/env python3
"""ws_check — проверка моста rosbridge по websocket (как его видит симулятор).

Подключается к URL (ws:// или wss://), подписывается на /result/velocity и
/result/position (rosbridge v2, JSON), считает сообщения за --seconds и
печатает частоту и последний выход. Код 0 — пришло хотя бы --min сообщений.

    python3 tools/ws_check.py ws://localhost:9090
    python3 tools/ws_check.py wss://demo.example.org/ros --seconds 10
    python3 tools/ws_check.py ws://proxy/ros --insecure      # самоподписанный сертификат

Нужен tornado (зависимость rosbridge_server, есть в образе vectra/tram).
"""

import argparse
import asyncio
import json
import ssl
import sys
import time


async def run(a):
    from tornado.httpclient import HTTPRequest
    from tornado.websocket import websocket_connect
    kw = {}
    if a.url.startswith("wss://") and a.insecure:
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        kw["ssl_options"] = ctx
    req = HTTPRequest(a.url, headers={"Origin": a.origin} if a.origin else None,
                      connect_timeout=a.timeout, request_timeout=a.timeout, **kw)
    t0 = time.monotonic()
    ws = await websocket_connect(req)
    print(f"[ws_check] подключено к {a.url} за {time.monotonic() - t0:.2f} с", flush=True)
    for topic, typ in (("/result/velocity", "tram_vehicle_msgs/msg/VelocitySensor"),
                       ("/result/position", "nav_msgs/msg/Odometry")):
        await ws.write_message(json.dumps({"op": "subscribe", "topic": topic, "type": typ,
                                           "queue_length": 50}))
    n = {"/result/velocity": 0, "/result/position": 0}
    last = {}
    first = None
    t_end = time.monotonic() + a.seconds
    while time.monotonic() < t_end:
        try:
            raw = await asyncio.wait_for(ws.read_message(), timeout=max(0.1, t_end - time.monotonic()))
        except asyncio.TimeoutError:
            break
        if raw is None:
            print("[ws_check] соединение закрыто сервером", flush=True)
            break
        m = json.loads(raw)
        if m.get("op") == "publish" and m.get("topic") in n:
            n[m["topic"]] += 1
            last[m["topic"]] = m["msg"]
            first = first or time.monotonic()
        elif m.get("op") == "status":
            print(f"[ws_check] статус моста: {m.get('level')} {m.get('msg')}", flush=True)
    ws.close()
    span = (time.monotonic() - first) if first else 0.0
    for topic, c in n.items():
        rate = (c - 1) / span if c > 1 and span > 0 else 0.0
        print(f"[ws_check] {topic}: {c} сообщений, {rate:.1f} Гц", flush=True)
    if "/result/velocity" in last:
        v = last["/result/velocity"]
        print(f"[ws_check] последняя скорость {v['velocity']:.3f} м/с, stamp "
              f"{v['header']['stamp']['sec']}.{v['header']['stamp']['nanosec']:09d}", flush=True)
    if "/result/position" in last:
        p = last["/result/position"]["pose"]["pose"]["position"]
        print(f"[ws_check] последнее положение x={p['x']:.2f} y={p['y']:.2f} z={p['z']:.2f} "
              f"({last['/result/position']['header']['frame_id']})", flush=True)
    return 0 if n["/result/velocity"] >= a.min else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("url", nargs="?", default="ws://localhost:9090")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--min", type=int, default=1, help="минимум сообщений скорости для успеха")
    ap.add_argument("--timeout", type=float, default=10.0, help="таймаут подключения, с")
    ap.add_argument("--origin", default="", help="заголовок Origin (проверка кросс-доменного доступа)")
    ap.add_argument("--insecure", action="store_true", help="не проверять сертификат wss")
    a = ap.parse_args()
    try:
        return asyncio.run(run(a))
    except Exception as e:                  # noqa: BLE001
        print(f"[ws_check] ОШИБКА: {type(e).__name__}: {e}", flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
