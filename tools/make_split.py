"""Разбиение прогонов на обучение и проверку по уникальным записям.

Исходное разбиение: sorted(ids)[2::5] - проверка (analysis/calib_drive.py).
В данных 25 пар побайтных дублей (docs/DATA.md). Правило:
  * дубль с копией в обучении исключается из проверки (утечка);
  * из пары, целиком попавшей в проверку, остаётся одна запись;
  * для метрик положения нужна GNSS; прогоны без GNSS остаются в проверке скорости? нет -
    без GNSS нет эталона ни скорости, ни положения, поэтому они в метриках не участвуют.
Результат: tools/split.json (фиксирован и коммитится; все инструменты читают его).
"""
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else ROOT / "out/data/bags.csv", encoding="utf-8")))
ids = sorted(r["bag"] for r in rows)
val0 = set(ids[2::5])
by = {r["bag"]: r for r in rows}
h = {b: by[b]["hash_full"] for b in ids}
groups = {}
for b in ids:
    groups.setdefault(h[b], []).append(b)
train = [b for b in ids if b not in val0]
train_hashes = {h[b] for b in train}
holdout, excluded = [], {}
seen = set()
for b in sorted(val0):
    if h[b] in train_hashes:
        excluded[b] = "копия есть в обучении"
        continue
    if h[b] in seen:
        excluded[b] = "дубль другой проверочной записи"
        continue
    seen.add(h[b])
    holdout.append(b)
gnss = {b for b in ids if int(by[b]["n_mvel"] or 0) >= 50}  # как evaluate.py: >= 50 отсчётов скорости GNSS
holdout_gnss = [b for b in holdout if b in gnss]
train_unique = []
tseen = set()
for b in train:
    if h[b] not in tseen:
        tseen.add(h[b]); train_unique.append(b)
out = {
    "_doc": "Разбиение по уникальным записям. holdout_scored - проверка метрик (есть GNSS). "
            "Лист и карта для оценки строятся только по train; для жюри - по всем данным.",
    "rule": "partner sorted(ids)[2::5] minus duplicates (hash_full from tools/data_audit.py)",
    "train": train, "train_unique": train_unique,
    "holdout": holdout, "holdout_scored": holdout_gnss,
    "excluded_from_holdout": excluded,
    "dup_groups": [g for g in groups.values() if len(g) > 1],
}
json.dump(out, open(ROOT / "tools/split.json", "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print(f"train {len(train)} (unique {len(train_unique)}), holdout {len(holdout)}, scored {len(holdout_gnss)}, excluded {excluded}")
print("scored:", holdout_gnss)
