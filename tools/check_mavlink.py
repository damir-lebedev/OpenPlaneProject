#!/usr/bin/env python3
"""Сверка телеметрии OpenPlane эталонным декодером MAVLink (pymavlink).

Поток байтов пишет тест test_mavlink:
    OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
    python3 tools/check_mavlink.py /tmp/tlm.bin

Каждый кадр должен разобраться с верной CRC; печатается сводка по
сообщениям и последние значения — так видно то же, что увидит
QGroundControl / Mission Planner. Нужен pip install pymavlink.
"""
import collections
import sys

from pymavlink.dialects.v20 import ardupilotmega as mavlink


def main(path):
    data = open(path, 'rb').read()
    mav = mavlink.MAVLink(None)
    mav.robust_parsing = False
    counts = collections.Counter()
    last = {}
    errors = 0
    for i in range(len(data)):
        try:
            msg = mav.parse_char(data[i:i + 1])
        except mavlink.MAVError as error:
            errors += 1
            print('ошибка разбора:', error)
            continue
        if msg is None:
            continue
        if msg.get_type() == 'BAD_DATA':
            errors += 1
            print('битый кадр:', msg)
            continue
        counts[msg.get_type()] += 1
        last[msg.get_type()] = msg

    print(f'{len(data)} байт, {sum(counts.values())} кадров, ошибок: {errors}')
    for name, n in sorted(counts.items()):
        print(f'  {name:24s} {n:5d}   {last[name]}')
    hb = last.get('HEARTBEAT')
    if hb is not None:
        print('Режим так, как его покажет GCS:', mavlink.enums['PLANE_MODE'][hb.custom_mode].name)
    return 1 if errors or not counts else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else '/tmp/tlm.bin'))
