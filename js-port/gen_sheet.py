#!/usr/bin/env python3
"""Лист параметров песочницы из листа пакета: simulator/js/sheet.js.

Песочница считает оценку JS-портом ядра (js-port/est.js) с тем же листом, что
нода ROS 2 у жюри: config/tram.yaml пакета (2 тележки, км/ч, табличный привод
A(u, v), калиброванные шумы и σ). Лист читается так же, как его читают нода и
tools/eval.py (tools/export_replay.resolve_sheet -> Params пакета), и
пишется целиком (все поля Params) плюс параметры ноды, которые видит ядро
(таймауты входов, окно выставки).

    docker run --rm -v <repo>:/repo -w /repo vectra/tram:integration \
        python3 js-port/gen_sheet.py [--sheet jury|eval|<путь>] [--out simulator/js/sheet.js]

Перезапускать после каждой правки листа (tools/gen_params.py) — иначе
simulator/test/page_test.js и js-port/check_sheet.js сообщат о расхождении
отпечатка листа.
"""
import argparse
import dataclasses
import hashlib
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import export_replay as X  # noqa: E402

NODE_KEYS = ("wheel_timeout_s", "handle_timeout_s", "init_window_s")


def sha1_lf(path):
    with open(path, "rb") as fh:
        return hashlib.sha1(fh.read().replace(b"\r\n", b"\n")).hexdigest()[:12]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sheet", default="jury")
    ap.add_argument("--out", default=os.path.join(ROOT, "simulator", "js", "sheet.js"))
    a = ap.parse_args()
    params, node, info = X.resolve_sheet(a.sheet)
    path = str(info.get("path") or os.path.join(X.PKG, "config", "tram.yaml"))
    path = path if os.path.isabs(path) else os.path.join(ROOT, path)
    core = {k: (list(v) if isinstance(v, tuple) else v)
            for k, v in dataclasses.asdict(params).items()}
    rel = os.path.relpath(path, ROOT).replace(os.sep, "/")
    doc = dict(
        sheet=a.sheet, path=rel, label=info.get("label", rel),
        sheet_sha1=sha1_lf(path), core_sha1=X.core_sha1(),
        core=core, node={k: node[k] for k in NODE_KEYS if k in node},
    )
    body = json.dumps(doc, ensure_ascii=False, indent=1)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("// Сгенерировано js-port/gen_sheet.py из листа пакета — не править руками.\n")
        fh.write("(function (g) {\n")
        fh.write(f"g.TV_SHEET = {body};\n")
        fh.write("if (typeof module !== 'undefined') module.exports = g.TV_SHEET;\n")
        fh.write("})(typeof window !== 'undefined' ? window : globalThis);\n")
    print("ok", a.out, rel, doc["sheet_sha1"], "ядро", doc["core_sha1"],
          f"осей {core['n_axles']}, {core['meas_units']}, таблица {len(core['acc_table'])}")


if __name__ == "__main__":
    main()
