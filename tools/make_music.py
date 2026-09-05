#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Генератор фоновой музыки для l2d_m3g -> res/music.mid

Стиль: эмбиент лабораторий Aperture (Kelly Bailey / Portal). Никакой
функциональной гармонии и барабанной подложки: статичная модальная петля
в ля-миноре, стеклянные арпеджио, глубокий синтезаторный пульс и редкие
"приборные" блипы. Музыка задумана как бесконечная петля, поэтому
последний такт разрежается, чтобы стык не был слышен.

Дополнительно скрипт умеет отрисовать WAV-превью простым софтверным
синтезатором - чтобы можно было послушать, не запуская игру:

    python3 tools/make_music.py --preview music_preview.wav
"""

import array
import math
import struct
import sys

TPQ = 480                 # тиков на четверть (как в исходном music.mid)
BEAT = TPQ
BAR = 4 * BEAT
BPM = 84
BARS = 32

# --- каналы и тембры (General MIDI, номера уже 0-based) ---
CH_PAD, PRG_PAD = 0, 88      # Pad 1 (new age)
CH_ARP, PRG_ARP = 1, 8       # Celesta
CH_BASS, PRG_BASS = 2, 38    # Synth Bass 1
CH_LEAD, PRG_LEAD = 3, 11    # Vibraphone
CH_FX, PRG_FX = 4, 98        # FX 3 (crystal)
CH_DRUM = 9

# Гармония: четыре аккорда по два такта. Вводного тона нет намеренно -
# петля никуда не "разрешается" и может крутиться бесконечно.
#          бас   пэд(3 голоса)      пул для арпеджио
CHORDS = [
    (45, [57, 60, 64], [69, 72, 76, 83, 81]),   # Am9
    (41, [53, 57, 64], [65, 69, 72, 76, 72]),   # Fmaj7
    (38, [50, 57, 65], [69, 74, 77, 81, 74]),   # Dm9
    (40, [52, 59, 62], [67, 71, 74, 79, 71]),   # Em7
]

# Ритмические маски арпеджио (восьмые в такте)
ARP_MASKS = [
    [1, 0, 1, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 1, 0, 1],
    [1, 0, 0, 1, 1, 0, 1, 1],
    [1, 0, 1, 1, 0, 1, 1, 0],
]

# Мотив вибрафона: (такт, доля, нота, длительность в долях)
MOTIF = [
    (16, 0.0, 76, 2.0), (16, 2.0, 74, 1.0), (16, 3.0, 72, 1.0),
    (17, 0.0, 71, 3.0),
    (20, 0.0, 69, 2.0), (20, 2.0, 72, 1.0), (20, 3.0, 74, 1.0),
    (21, 0.0, 76, 3.0),
    (24, 0.0, 76, 2.0), (24, 2.0, 74, 1.0), (24, 3.0, 72, 1.0),
    (25, 0.0, 69, 3.5),
    (28, 0.0, 64, 2.0), (28, 2.0, 67, 2.0),
    (29, 0.0, 71, 3.5),
]

# Редкие блипы приборов: (такт, доля, нота)
BLIPS = [(11, 3.5, 96), (15, 2.5, 98), (19, 3.5, 93),
         (23, 1.5, 96), (27, 3.5, 100), (30, 2.5, 93)]


def notes():
    """Собирает всю партитуру: список (tick, channel, pitch, velocity, length)."""
    out = []

    for bar in range(BARS):
        t0 = bar * BAR
        chord = CHORDS[(bar // 2) % len(CHORDS)]
        root, pad, pool = chord
        new_chord = (bar % 2) == 0
        fade = bar >= BARS - 1          # последний такт - разрежаем

        # --- пэд: два такта на аккорд, с небольшим нахлёстом ---
        if new_chord:
            for i, p in enumerate(pad):
                vel = 52 if i == 0 else 44
                out.append((t0 + i * 12, CH_PAD, p, vel, 2 * BAR - 40))

        # --- бас: длинная нота от корня плюс тихий подтолчок на 4-ю долю ---
        if not fade:
            out.append((t0, CH_BASS, root, 62, int(3.5 * BEAT)))
            if bar % 2 == 1:
                out.append((t0 + 3 * BEAT, CH_BASS, root + 12, 40, BEAT // 2))

        # --- арпеджио: со второй трети, восьмыми по маске ---
        if 8 <= bar < BARS - 1:
            mask = ARP_MASKS[bar % len(ARP_MASKS)]
            step = 0
            for slot in range(8):
                if not mask[slot]:
                    continue
                pitch = pool[step % len(pool)]
                step += 1
                vel = 58 if slot == 0 else (46 if slot % 2 == 0 else 38)
                if bar >= 24:
                    vel += 6
                out.append((t0 + slot * (BEAT // 2), CH_ARP, pitch, vel, BEAT // 2 + 60))

        # --- пульс "машинерии": тихие тики, только в плотной части ---
        if 16 <= bar < BARS - 2:
            out.append((t0, CH_DRUM, 36, 46, 60))                     # мягкая бочка
            for beat in range(4):
                out.append((t0 + beat * BEAT + BEAT // 2, CH_DRUM, 42, 30, 40))
            if bar % 2 == 1:
                out.append((t0 + 2 * BEAT, CH_DRUM, 37, 34, 40))      # рим-шот

    # --- мотив ---
    for bar, beat, pitch, length in MOTIF:
        out.append((bar * BAR + int(beat * BEAT), CH_LEAD, pitch, 54,
                    int(length * BEAT)))

    # --- блипы ---
    for bar, beat, pitch in BLIPS:
        out.append((bar * BAR + int(beat * BEAT), CH_FX, pitch, 44, BEAT // 2))

    return out


# ---------------------------------------------------------------- MIDI

def vlq(value):
    """Переменная длина (variable length quantity)."""
    buf = [value & 0x7F]
    value >>= 7
    while value:
        buf.append((value & 0x7F) | 0x80)
        value >>= 7
    return bytes(reversed(buf))


def chunk(tag, body):
    return tag + struct.pack('>I', len(body)) + body


def build_track(events):
    """events: список (tick, порядок, bytes) -> тело трека."""
    events.sort(key=lambda e: (e[0], e[1]))
    body = b''
    prev = 0
    for tick, _, data in events:
        body += vlq(tick - prev) + data
        prev = tick
    return chunk(b'MTrk', body + b'\x00\xff\x2f\x00')


def write_midi(path):
    score = notes()

    # трек 0: темп и название
    tempo = int(round(60000000.0 / BPM))
    meta = [
        (0, 0, b'\xff\x03' + vlq(len(b'Aperture')) + b'Aperture'),
        (0, 1, b'\xff\x51\x03' + struct.pack('>I', tempo)[1:]),
        (0, 2, b'\xff\x58\x04\x04\x02\x18\x08'),
    ]

    # по треку на канал
    groups = {}
    for tick, ch, pitch, vel, length in score:
        groups.setdefault(ch, []).append((tick, ch, pitch, vel, length))

    programs = {CH_PAD: PRG_PAD, CH_ARP: PRG_ARP, CH_BASS: PRG_BASS,
                CH_LEAD: PRG_LEAD, CH_FX: PRG_FX}

    tracks = [build_track(meta)]

    for ch in sorted(groups):
        ev = []
        if ch in programs:
            ev.append((0, 0, bytes([0xC0 | ch, programs[ch]])))
        # немного реверберации и панорамы, если синтезатор их поддерживает
        ev.append((0, 1, bytes([0xB0 | ch, 91, 90 if ch != CH_DRUM else 40])))

        for tick, _, pitch, vel, length in groups[ch]:
            ev.append((tick, 2, bytes([0x90 | ch, pitch, vel])))
            ev.append((tick + length, 3, bytes([0x80 | ch, pitch, 0])))

        tracks.append(build_track(ev))

    header = chunk(b'MThd', struct.pack('>HHH', 1, len(tracks), TPQ))
    data = header + b''.join(tracks)

    with open(path, 'wb') as f:
        f.write(data)

    return data, score


# ------------------------------------------------------------- превью

SR = 16000


def render_wav(path, score):
    """Простенький софтверный синтез, только чтобы услышать композицию."""
    spb = 60.0 / BPM / TPQ                      # секунд на тик
    total = int((BARS * BAR * spb + 2.0) * SR)
    buf = array.array('d', [0.0]) * total

    for tick, ch, pitch, vel, length in score:
        start = int(tick * spb * SR)
        dur = length * spb
        amp = vel / 127.0

        if ch == CH_DRUM:
            render_drum(buf, start, pitch, amp)
            continue

        freq = 440.0 * (2.0 ** ((pitch - 69) / 12.0))
        if ch == CH_PAD:
            render_voice(buf, start, dur + 0.6, freq, amp * 0.16, 'saw', 0.45, 0.6)
        elif ch == CH_BASS:
            render_voice(buf, start, dur + 0.15, freq, amp * 0.5, 'saw', 0.02, 0.25)
        elif ch == CH_ARP:
            render_voice(buf, start, 1.1, freq, amp * 0.28, 'bell', 0.002, 1.0)
        elif ch == CH_LEAD:
            render_voice(buf, start, dur + 0.9, freq, amp * 0.3, 'bell', 0.004, 1.2)
        else:
            render_voice(buf, start, 0.5, freq, amp * 0.2, 'sine', 0.002, 0.45)

    # нормализация и запись
    peak = 0.0
    for v in buf:
        av = v if v > 0 else -v
        if av > peak:
            peak = av
    gain = 0.89 / peak if peak > 0 else 1.0

    pcm = array.array('h', [0]) * total
    for i in range(total):
        s = buf[i] * gain
        # мягкое ограничение
        s = math.tanh(s)
        pcm[i] = int(s * 32000)

    raw = pcm.tobytes()
    hdr = (b'RIFF' + struct.pack('<I', 36 + len(raw)) + b'WAVEfmt ' +
           struct.pack('<IHHIIHH', 16, 1, 1, SR, SR * 2, 2, 16) +
           b'data' + struct.pack('<I', len(raw)))
    with open(path, 'wb') as f:
        f.write(hdr + raw)


def render_voice(buf, start, dur, freq, amp, kind, attack, release):
    n = int(dur * SR)
    if start + n > len(buf):
        n = len(buf) - start
    if n <= 0:
        return

    step = 2.0 * math.pi * freq / SR
    det = 2.0 * math.pi * freq * 1.004 / SR
    lp = 0.0
    a = max(1, int(attack * SR))
    r = max(1, int(release * SR))

    for i in range(n):
        # огибающая: колокольчики затухают экспонентой, тянущиеся тембры
        # имеют атаку и спад в конце
        if i < a:
            env = i / a
        elif kind == 'bell':
            env = math.exp(-4.0 * (i - a) / n)
        else:
            tail = n - i
            env = 1.0 if tail > r else tail / r
        if env <= 0.0:
            continue

        ph = i * step
        if kind == 'saw':
            x = ph % (2.0 * math.pi) / math.pi - 1.0
            y = (x + (i * det % (2.0 * math.pi) / math.pi - 1.0)) * 0.5
            lp += 0.25 * (y - lp)          # простой ФНЧ, чтобы не резало слух
            y = lp
        elif kind == 'bell':
            y = math.sin(ph) + 0.35 * math.sin(2.0 * ph) + 0.12 * math.sin(3.01 * ph)
            y *= 0.7
        else:
            y = math.sin(ph)

        buf[start + i] += y * env * amp


def render_drum(buf, start, pitch, amp):
    if pitch == 36:                      # бочка
        n = int(0.18 * SR)
        for i in range(n):
            if start + i >= len(buf):
                break
            env = math.exp(-18.0 * i / n)
            f = 90.0 * math.exp(-6.0 * i / n) + 40.0
            buf[start + i] += math.sin(2.0 * math.pi * f * i / SR) * env * amp * 0.8
    else:                                # тики: короткий шум
        n = int(0.05 * SR)
        seed = 12345 + pitch
        for i in range(n):
            if start + i >= len(buf):
                break
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            noise = (seed / 0x3FFFFFFF) - 1.0
            env = math.exp(-40.0 * i / n)
            buf[start + i] += noise * env * amp * 0.25


def main():
    out = 'res/music.mid'
    data, score = write_midi(out)
    dur = BARS * BAR * (60.0 / BPM / TPQ)
    print('%s: %d байт, %d нот, %.1f с, %d BPM' %
          (out, len(data), len(score), dur, BPM))

    if '--preview' in sys.argv:
        path = sys.argv[sys.argv.index('--preview') + 1]
        render_wav(path, score)
        print('превью: ' + path)


if __name__ == '__main__':
    main()
