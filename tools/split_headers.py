#!/usr/bin/env python3
"""Header-only -> .h + .cpp: разделение кода прошивки для ветки
feature/split-headers.

Основная ветка — header-only: одна единица трансляции, компилятор видит
всё сразу. Для тех, кто привык к классической раскладке, есть ветка с
реализацией в .cpp. Чтобы ветки не расходились, вторая не пишется руками,
а генерируется этим скриптом из первой:

    git checkout -B feature/split-headers <основная ветка>
    python3 tools/split_headers.py          # переписывает include/, создаёт src/core/
    pio test -e native -e native-stm32 && tools/build_matrix.sh

Что делает. libclang разбирает каждый заголовок include/ (с фейками из
test/native/support, как нативные тесты) и находит определения функций
внутри заголовка. Функция переезжает в src/core/<путь>.cpp, в заголовке
остаётся объявление, если:
  - тело многострочное (однострочные геттеры остаются в классе — так
    принято в C++ и так компилятор может их встроить без LTO);
  - это не шаблон, не constexpr, не оператор, не = default/delete;
  - определение не внутри #if заголовка (иначе .cpp потерял бы условие).
В .cpp у метода убираются virtual/static/explicit/override и значения
параметров по умолчанию; тип результата уходит в хвост
(auto Класс::метод(...) -> Тип) — тогда вложенные типы класса не нужно
квалифицировать.

Нужен libclang: pip install libclang (версия как у системного clang).
"""
import os
import re
import subprocess
import sys
from collections import OrderedDict

import clang.cindex as ci

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
INCLUDE = os.path.join(ROOT, "include")
OUT = os.path.join(ROOT, "src", "core")



def clang_resource_args():
    """libclang из pip не знает, где лежат stddef.h и прочие встроенные
    заголовки компилятора, — берём каталог у системного clang."""
    try:
        resource = subprocess.run(["clang", "-print-resource-dir"], capture_output=True,
                                  text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return []
    return ["-isystem", os.path.join(resource, "include")]


ESP32_ARGS = ["-x", "c++", "-std=gnu++17", "-DBOARD_ESP32_S3",
              "-I" + INCLUDE, "-I" + os.path.join(ROOT, "test/native/support")]
STM32_ARGS = ["-x", "c++", "-std=gnu++17", "-DBOARD_STM32H743",
              "-I" + os.path.join(INCLUDE, "hal/stm32/compat"), "-I" + INCLUDE,
              "-I" + os.path.join(ROOT, "test/native/support_stm32"),
              "-I" + os.path.join(ROOT, "test/native/support")]
ESP32_ARGS += clang_resource_args()
STM32_ARGS += clang_resource_args()

# Эти заголовки — только константы, таблицы и макросы.
SKIP = {"config/Config.h", "config/Channels.h", "config/Controls.h", "sensors/SensorSelection.h",
        "telemetry/WebDashboardPage.h"}

METHOD_KINDS = {ci.CursorKind.CXX_METHOD, ci.CursorKind.CONSTRUCTOR, ci.CursorKind.DESTRUCTOR}
CLASS_KINDS = {ci.CursorKind.CLASS_DECL, ci.CursorKind.STRUCT_DECL}
DROP_PREFIX = re.compile(r"\b(virtual|static|inline|explicit)\b\s*")
DROP_SUFFIX = re.compile(r"\b(override|final)\b")


def conditional_lines(text):
    """Строки внутри #if ... #endif."""
    inside = set()
    depth = 0
    for number, line in enumerate(text.split("\n"), start=1):
        stripped = line.strip()
        if re.match(r"#\s*(if|ifdef|ifndef)\b", stripped):
            depth += 1
        elif re.match(r"#\s*endif\b", stripped):
            depth -= 1
        elif depth > 0:
            inside.add(number)
    return inside


def owner_chain(cursor):
    """(классы, пространства имён) от внешнего к внутреннему; None — не переносим."""
    classes, namespaces = [], []
    parent = cursor.semantic_parent
    while parent is not None and parent.kind != ci.CursorKind.TRANSLATION_UNIT:
        if parent.kind in CLASS_KINDS:
            classes.append(parent.spelling)
        elif parent.kind == ci.CursorKind.NAMESPACE:
            if not parent.spelling:
                return None   # анонимное пространство имён
            namespaces.append(parent.spelling)
        else:
            return None       # шаблон класса, локальный класс в функции...
        parent = parent.semantic_parent
    return list(reversed(classes)), list(reversed(namespaces))


def matching_paren(tokens, start):
    depth = 0
    for i in range(start, len(tokens)):
        if tokens[i].spelling == "(":
            depth += 1
        elif tokens[i].spelling == ")":
            depth -= 1
            if depth == 0:
                return i
    return None


def default_arg_ranges(cursor):
    """Диапазоны '= значение' в параметрах (в .cpp не повторяются)."""
    ranges = []
    for param in cursor.get_arguments():
        depth = 0
        for tok in param.get_tokens():
            if tok.spelling in "([{<":
                depth += 1
            elif tok.spelling in ")]}>":
                depth -= 1
            elif tok.spelling == "=" and depth == 0:
                ranges.append((tok.extent.start.offset, param.extent.end.offset))
                break
    return ranges


def dedent(text, column):
    lines = text.split("\n")
    out = [lines[0]]
    for line in lines[1:]:
        cut = 0
        while cut < column and cut < len(line) and line[cut] == " ":
            cut += 1
        out.append(line[cut:])
    return "\n".join(out)


def split_function(cursor, raw, conditional):
    """(замена в заголовке, определение для .cpp, пространства имён) или None.

    raw — байты файла: смещения libclang — в байтах, а в комментариях
    кириллица (2 байта на символ).
    """
    def text(a, b):
        return raw[a:b].decode("utf-8")

    if cursor.kind == ci.CursorKind.FUNCTION_DECL and cursor.storage_class == ci.StorageClass.STATIC:
        return None
    if cursor.spelling.startswith("operator"):
        return None
    ext = cursor.extent
    if ext.start.line == ext.end.line or ext.start.line in conditional:
        return None
    chain = owner_chain(cursor)
    if chain is None:
        return None
    classes, namespaces = chain
    if cursor.kind in METHOD_KINDS and not classes:
        return None

    tokens = list(cursor.get_tokens())
    name_off = cursor.location.offset
    try:
        paren = next(i for i, t in enumerate(tokens) if t.spelling == "(" and t.extent.start.offset > name_off)
    except StopIteration:
        return None
    close = matching_paren(tokens, paren)
    if close is None:
        return None
    k = close + 1
    while k < len(tokens) and tokens[k].spelling not in ("{", ":", "=", ";", "try"):
        k += 1
    if k >= len(tokens) or tokens[k].spelling not in ("{", ":"):
        return None
    head = [t.spelling for t in tokens[:paren]]
    if "constexpr" in head or "friend" in head or "template" in head:
        return None

    start = ext.start.offset
    cut = tokens[k].extent.start.offset
    end = ext.end.offset
    close_end = tokens[close].extent.end.offset

    declaration = text(start, cut).rstrip()
    if cursor.kind == ci.CursorKind.FUNCTION_DECL:
        declaration = re.sub(r"\binline\s+", "", declaration, count=1)
    header_replacement = declaration + ";"

    ret = DROP_PREFIX.sub("", text(start, name_off)).strip()
    params_raw = raw[name_off:close_end]
    for a, b in sorted(default_arg_ranges(cursor), reverse=True):
        params_raw = params_raw[: a - name_off].rstrip() + params_raw[b - name_off:]
    params = params_raw.decode("utf-8")
    quals = DROP_SUFFIX.sub("", text(close_end, cut)).strip()
    quals = (" " + quals) if quals else ""
    body = dedent(text(cut, end), ext.start.column - 1)

    if cursor.kind == ci.CursorKind.FUNCTION_DECL:
        definition = f"{ret} {params}{quals}\n{body}"
    else:
        qual = "::".join(classes) + "::"
        if ret:
            definition = f"auto {qual}{params}{quals} -> {ret}\n{body}"
        else:
            definition = f"{qual}{params}{quals}\n{body}"
    return (start, end, header_replacement), definition, tuple(namespaces)


def process(rel):
    path = os.path.join(INCLUDE, rel)
    raw = open(path, "rb").read()
    text = raw.decode("utf-8")
    args = STM32_ARGS if rel.startswith("hal/stm32/") else ESP32_ARGS
    tu = ci.Index.create().parse(path, args=args)
    errors = [d for d in tu.diagnostics if d.severity >= ci.Diagnostic.Error]
    if errors:
        print(f"  ! {rel}: не разобрался ({errors[0].spelling}) — оставлен header-only")
        return 0

    conditional = conditional_lines(text)
    edits, defs = [], []
    seen = set()
    for cursor in tu.cursor.walk_preorder():
        if cursor.kind not in METHOD_KINDS and cursor.kind != ci.CursorKind.FUNCTION_DECL:
            continue
        if not cursor.is_definition() or cursor.location.file is None:
            continue
        if os.path.abspath(cursor.location.file.name) != path:
            continue
        key = (cursor.extent.start.offset, cursor.extent.end.offset)
        if key in seen:
            continue
        seen.add(key)
        result = split_function(cursor, raw, conditional)
        if result:
            edit, definition, namespaces = result
            edits.append(edit)
            defs.append((edit[0], namespaces, definition))

    if not edits:
        return 0

    for start, end, replacement in sorted(edits, reverse=True):
        raw = raw[:start] + replacement.encode("utf-8") + raw[end:]
    open(path, "wb").write(raw)

    groups = OrderedDict()
    for _, namespaces, definition in sorted(defs):
        groups.setdefault(namespaces, []).append(definition)

    out = [f"// Реализация {rel}: вынесена из заголовка tools/split_headers.py",
           "// (ветка feature/split-headers). Правки делайте в основной ветке и",
           "// перегенерируйте — так две раскладки кода не расходятся.",
           "",
           f'#include "{rel}"',
           ""]
    for namespaces, items in groups.items():
        for ns in namespaces:
            out.append(f"namespace {ns}\n{{")
        out.append("")
        out.append("\n\n".join(items))
        out.append("")
        for ns in reversed(namespaces):
            out.append(f"}}  // namespace {ns}")
        out.append("")
    cpp = os.path.join(OUT, os.path.splitext(rel)[0] + ".cpp")
    os.makedirs(os.path.dirname(cpp), exist_ok=True)
    open(cpp, "w", encoding="utf-8").write("\n".join(out).rstrip() + "\n")
    return len(edits)


def main():
    headers = []
    for directory, _, files in os.walk(INCLUDE):
        for name in files:
            if name.endswith(".h"):
                headers.append(os.path.relpath(os.path.join(directory, name), INCLUDE))
    total = 0
    for rel in sorted(headers):
        if rel in SKIP:
            continue
        moved = process(rel)
        if moved:
            print(f"  {rel}: {moved} функций -> src/core/{os.path.splitext(rel)[0]}.cpp")
        total += moved
    print(f"Перенесено функций: {total}")


if __name__ == "__main__":
    sys.exit(main())
