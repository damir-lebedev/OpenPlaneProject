#!/usr/bin/env python3
"""
PLAN_COPTER.md -> tools/plan_tracker/items.json для трекера плана
(страница tools/plan_tracker/index.html, опубликована артефактом).

    python tools/plan_to_json.py

Пункт плана: «### [ ] 001 · Заголовок — Стенд: да», дальше строки
«- **Поле:** значение» (несколько полей в строке — через « · »).
Статус из файла ([ ] [~] [x] [!]) — запасной: живой статус трекер берёт
из своей базы (коллекция status), её обновляет Claude после каждого батча.
"""

import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PLAN = os.path.join(ROOT, "PLAN_COPTER.md")
OUT = os.path.join(HERE, "plan_tracker", "items.json")

STATUS = {" ": "todo", "~": "doing", "x": "done", "!": "blocked"}
FIELDS = {
    "BF": "bf",
    "Что": "what",
    "Сейчас": "now",
    "Файлы": "files",
    "Тест": "test",
    "UI": "ui",
    "Итог": "result",
}
ITEM = re.compile(r"^### \[(.)\] (\S+) · (.+)$")
SECTION = re.compile(r"^## (\d+)\. (.+)$")
FIELD = re.compile(r"\*\*([^*]+):\*\*\s*")


def parse_fields(line, item):
    # «- **Файлы:** a · **Тест:** b» -> несколько полей
    body = line[2:].strip()
    parts = FIELD.split(body)
    # parts: ['', key1, value1, key2, value2, ...]
    for i in range(1, len(parts) - 1, 2):
        key = FIELDS.get(parts[i].strip())
        value = parts[i + 1].strip()
        value = re.sub(r"\s*·\s*$", "", value)
        if key:
            item[key] = (item.get(key, "") + " " + value).strip() if key in item else value


def ui_tab(item):
    if item["repo"] == "studio":
        return "Приложение"
    ui = item.get("ui", "")
    m = re.search(r"«([^»→]+)", ui)
    if m:
        tab = m.group(1).strip()
        return "Моторы" if "ESC" in tab else tab
    if "режим" in ui:
        return "Режимы"
    if "строка состояния" in ui:
        return "Состояние"
    return ""


def main():
    sections, items = [], []
    section = None
    with open(PLAN, encoding="utf-8") as f:
        lines = f.read().split("\n")
    for line in lines:
        m = SECTION.match(line)
        if m:
            section = {"id": int(m.group(1)), "title": m.group(2).strip()}
            sections.append(section)
            continue
        m = ITEM.match(line)
        if m:
            title = m.group(3).strip()
            bench = "Стенд: да" in title
            repo = "studio" if "Репо: studio" in title else "firmware"
            title = re.sub(r"\s+—\s+(Стенд: да|Репо: studio)", "", title)
            items.append({
                "id": m.group(2),
                "section": section["id"] if section else 0,
                "title": title,
                "fileStatus": STATUS.get(m.group(1), "todo"),
                "bench": bench,
                "repo": repo,
                "docs": m.group(2).startswith("D"),
            })
            continue
        if items and line.startswith("- **"):
            parse_fields(line, items[-1])
    for it in items:
        names = []
        for chunk in re.findall(r"`([^`]+)`", it.get("bf", "")):
            for word in chunk.split():
                if re.fullmatch(r"[a-z][a-z0-9_]*", word) and word not in names:
                    names.append(word)
        it["settings"] = names
        it["tab"] = ui_tab(it)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        json.dump({"sections": sections, "items": items}, f, ensure_ascii=False, indent=1)
    print("{} разделов, {} пунктов -> {}".format(len(sections), len(items), os.path.relpath(OUT, ROOT)))


if __name__ == "__main__":
    main()
