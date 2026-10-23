#pragma once

// ============================================================
// GETFUN F405V3 (стек «F405V3 + 100A ESC», продавец WXL): STM32F405RGT6,
// ICM42688P, SPL06, флеш 16 МБ, OSD, 6 UART, M1–M8, PINIO USER1 (камеры) и
// USER2 (12 В). Родная прошивка — Betaflight «GETFUNF405V3», но такого
// таргета нет ни в одном репозитории Betaflight: распиновка (какой выход на
// какой ножке STM32) известна только самой плате.
//
// Этот файл заменит tools/bf_board.py по дампу с платы — пока он заглушка:
//   1. плата с родным Betaflight, Configurator закрыт:
//        python tools/bf_board.py read --port COMx --out stack/GETFUNF405V3_bf.txt
//   2. плата в DFU (зажать BOOT, вставить USB) — сохранить родную прошивку:
//        python tools/bf_board.py backup --out stack/GETFUNF405V3_original.bin
//   3. заголовок платы:
//        python tools/bf_board.py header stack/GETFUNF405V3_bf.txt --baro SPL06 --out include/config/boards/GETFUNF405V3.h
// (docs/COPTER.md, «Полётник на F405 из-под Betaflight»). Сообщение ниже —
// латиницей: кириллицу в #error компилятор печатает кодами.
// ============================================================

#error "GETFUN F405V3: no pin map yet - dump it from the board first: python tools/bf_board.py read --port COMx --out stack/GETFUNF405V3_bf.txt, then backup and header (see include/config/boards/GETFUNF405V3.h)"
