#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Генератор фоновой музыки для l2d_m3g.

Две композиции:

  aperture - холодный эмбиент лабораторий (Kelly Bailey / Portal):
             статичная модальная петля, стеклянные арпеджио, тихая
             "машинерия". Лежит в assets/music/aperture_ambient.mid.

  stalker  - тяжёлый dark ambient в духе MoozE: очень медленный,
             тягучий, фригийский лад, гудящий бас, смычковый кластер
             с малой секундой, тритоновый нагнетающий наплыв и редкие
             колокольные удары. Это то, что играет в игре (res/music.mid).

Использование:
    python3 tools/make_music.py                       # обе, по своим путям
    python3 tools/make_music.py --preview out.wav     # + WAV-превью игровой
    python3 tools/make_music.py --style aperture --preview a.wav

WAV-превью рисуется своим примитивным синтезатором - только чтобы
услышать материал; в игре звучит синтезатор General MIDI устройства.
"""

import array
import math
import struct
import sys

TPQ = 480                      # тиков на четверть
BEAT = TPQ
BAR = 4 * BEAT

# ------------------------------------------------------------------ утилиты


class Score(object):
    """Партитура: ноты, контроллеры, тембры, темп."""

    def __init__(self, bpm, bars, programs, name):
        self.bpm = bpm
        self.bars = bars
        self.programs = programs      # {канал: GM-программа (0-based)}
        self.name = name
        self.notes = []               # (tick, ch, pitch, vel, length)
        self.ccs = []                 # (tick, ch, cc, value)
        self.voices = {}              # {канал: тембр для превью}

    def note(self, bar, beat, ch, pitch, vel, beats):
        self.notes.append((int(bar * BAR + beat * BEAT), ch, pitch, vel,
                           int(beats * BEAT)))

    def cc(self, bar, beat, ch, num, value):
        self.ccs.append((int(bar * BAR + beat * BEAT), ch, num,
                         max(0, min(127, int(value)))))

    def swell(self, ch, bar0, bar1, v0, v1, steps=12):
        """Плавное изменение экспрессии (CC11) - основа наплывов."""
        for i in range(steps + 1):
            k = i / float(steps)
            self.cc(bar0 + (bar1 - bar0) * k, 0, ch, 11, v0 + (v1 - v0) * k)

    def seconds(self):
        return self.bars * 4 * 60.0 / self.bpm


# ------------------------------------------------------- композиция 1: Portal

def score_aperture():
    CH_PAD, CH_ARP, CH_BASS, CH_LEAD, CH_FX, CH_DRUM = 0, 1, 2, 3, 4, 9

    s = Score(84, 32, {CH_PAD: 88, CH_ARP: 8, CH_BASS: 38,
                       CH_LEAD: 11, CH_FX: 98}, 'Aperture')
    s.voices = {CH_PAD: 'pad', CH_ARP: 'bell', CH_BASS: 'bass',
                CH_LEAD: 'bell', CH_FX: 'sine'}

    chords = [
        (45, [57, 60, 64], [69, 72, 76, 83, 81]),   # Am9
        (41, [53, 57, 64], [65, 69, 72, 76, 72]),   # Fmaj7
        (38, [50, 57, 65], [69, 74, 77, 81, 74]),   # Dm9
        (40, [52, 59, 62], [67, 71, 74, 79, 71]),   # Em7
    ]
    masks = [[1, 0, 1, 1, 0, 1, 0, 1], [1, 0, 1, 0, 1, 1, 0, 1],
             [1, 0, 0, 1, 1, 0, 1, 1], [1, 0, 1, 1, 0, 1, 1, 0]]

    for bar in range(s.bars):
        root, pad, pool = chords[(bar // 2) % len(chords)]
        fade = bar >= s.bars - 1

        if bar % 2 == 0:
            for i, p in enumerate(pad):
                s.note(bar, i * 0.025, CH_PAD, p, 52 if i == 0 else 44, 8 - 0.1)

        if not fade:
            s.note(bar, 0, CH_BASS, root, 62, 3.5)
            if bar % 2 == 1:
                s.note(bar, 3, CH_BASS, root + 12, 40, 0.5)

        if 8 <= bar < s.bars - 1:
            mask = masks[bar % len(masks)]
            step = 0
            for slot in range(8):
                if not mask[slot]:
                    continue
                vel = 58 if slot == 0 else (46 if slot % 2 == 0 else 38)
                if bar >= 24:
                    vel += 6
                s.note(bar, slot * 0.5, CH_ARP, pool[step % len(pool)], vel, 0.62)
                step += 1

        if 16 <= bar < s.bars - 2:
            s.note(bar, 0, CH_DRUM, 36, 46, 0.12)
            for beat in range(4):
                s.note(bar, beat + 0.5, CH_DRUM, 42, 30, 0.08)
            if bar % 2 == 1:
                s.note(bar, 2, CH_DRUM, 37, 34, 0.08)

    motif = [(16, 0, 76, 2), (16, 2, 74, 1), (16, 3, 72, 1), (17, 0, 71, 3),
             (20, 0, 69, 2), (20, 2, 72, 1), (20, 3, 74, 1), (21, 0, 76, 3),
             (24, 0, 76, 2), (24, 2, 74, 1), (24, 3, 72, 1), (25, 0, 69, 3.5),
             (28, 0, 64, 2), (28, 2, 67, 2), (29, 0, 71, 3.5)]
    for bar, beat, pitch, length in motif:
        s.note(bar, beat, CH_LEAD, pitch, 54, length)

    for bar, beat, pitch in [(11, 3.5, 96), (15, 2.5, 98), (19, 3.5, 93),
                             (23, 1.5, 96), (27, 3.5, 100), (30, 2.5, 93)]:
        s.note(bar, beat, CH_FX, pitch, 44, 0.5)

    return s


# ------------------------------------------------- композиция 2: dark ambient

def score_stalker():
    """
    Ре-фригийский: ре - ми-бемоль - фа - соль - ля - си-бемоль - до.
    Малая секунда над тоникой даёт то самое давящее звучание, а тритон
    (ля-бемоль) во второй половине добавляет безысходности. Метра как
    такового нет: такты нужны только как сетка времени, 4.6 секунды каждый.
    """
    CH_DRONE, CH_BOW, CH_BELL, CH_CHOIR, CH_METAL, CH_DRUM = 0, 1, 2, 3, 4, 9

    s = Score(52, 40, {CH_DRONE: 39,    # Synth Bass 2 - гул
                       CH_BOW: 92,      # Pad 5 (bowed) - смычковый пласт
                       CH_BELL: 14,     # Tubular Bells - далёкие удары
                       CH_CHOIR: 91,    # Pad 4 (choir) - хоровой наплыв
                       CH_METAL: 93},   # Pad 6 (metallic) - тритоновый скрежет
              'Zone')
    s.voices = {CH_DRONE: 'sub', CH_BOW: 'bow', CH_BELL: 'toll',
                CH_CHOIR: 'choir', CH_METAL: 'metal'}

    # --- нижний гул: держится всю петлю, перекладывается каждые 4 такта ---
    for bar in range(0, s.bars, 4):
        vel = 104 if (bar // 4) % 2 == 0 else 94
        s.note(bar, 0, CH_DRONE, 38, vel, 16.4)          # D2
        if bar >= 8:
            s.note(bar, 0.5, CH_DRONE, 26, vel - 18, 15.6)   # D1 - подпол
    s.swell(CH_DRONE, 0, 2, 100, 124)
    s.swell(CH_DRONE, 30, 39.5, 118, 88)

    # --- смычковый пласт: чистая квинта, потом кластер с малой секундой ---
    # вступительный пласт: без него первые полминуты петли - почти тишина
    s.note(0, 0, CH_BOW, 50, 92, 16.5)                   # D3
    s.note(0, 0.25, CH_BOW, 57, 84, 16.2)                # A3

    for bar in range(4, 36, 8):
        s.note(bar, 0, CH_BOW, 50, 96, 32.5)             # D3
        s.note(bar, 0.25, CH_BOW, 57, 88, 32.2)          # A3
        s.note(bar, 0.75, CH_BOW, 62, 72, 32.0)          # D4 - чтобы пласт
        if bar >= 12:                                    #      был слышен
            # ми-бемоль: та самая давящая секунда
            s.note(bar, 0.5, CH_BOW, 63, 78, 31.8)       # Eb4
    s.cc(0, 0, CH_BOW, 11, 96)
    s.swell(CH_BOW, 4, 8, 96, 112)
    s.swell(CH_BOW, 12, 16, 108, 127)
    s.swell(CH_BOW, 28, 34, 127, 92)
    s.swell(CH_BOW, 36, 39.5, 92, 62)

    # --- редкие колокольные удары: сигнал откуда-то издалека ---
    for bar, pitch, vel in [(3, 62, 100), (8, 62, 104), (14, 57, 92), (20, 63, 100),
                            (26, 50, 96), (32, 62, 108), (37, 57, 84)]:
        s.note(bar, 0, CH_BELL, pitch, vel, 6)
    s.cc(0, 0, CH_BELL, 11, 122)

    # --- хор: медленный нисходящий фригийский оборот, по два такта на ноту ---
    choir = [(16, 62), (18, 63), (20, 62), (22, 60),
             (24, 58), (26, 57), (30, 55), (32, 57)]
    for bar, pitch in choir:
        s.note(bar, 0, CH_CHOIR, pitch, 88, 8.5)
    s.cc(0, 0, CH_CHOIR, 11, 50)
    s.swell(CH_CHOIR, 15, 18, 50, 116)
    s.swell(CH_CHOIR, 26, 34, 116, 72)

    # --- тритон: нагнетание к концу петли и распад ---
    s.note(22, 0, CH_METAL, 56, 82, 16.5)                # Ab3
    s.note(22, 0.5, CH_METAL, 44, 74, 16.0)              # Ab2
    s.note(33, 0, CH_METAL, 56, 76, 12.0)
    s.cc(0, 0, CH_METAL, 11, 40)
    s.swell(CH_METAL, 22, 26, 40, 104)
    s.swell(CH_METAL, 27, 32.5, 104, 40)
    s.swell(CH_METAL, 33, 36, 40, 92)
    s.swell(CH_METAL, 36, 39, 92, 40)

    # --- два глухих удара где-то вдалеке ---
    s.note(12, 0, CH_DRUM, 41, 92, 0.5)
    s.note(28, 2, CH_DRUM, 41, 84, 0.5)

    return s


# ---------------------------------------------------------------- запись MIDI

def vlq(value):
    buf = [value & 0x7F]
    value >>= 7
    while value:
        buf.append((value & 0x7F) | 0x80)
        value >>= 7
    return bytes(reversed(buf))


def chunk(tag, body):
    return tag + struct.pack('>I', len(body)) + body


def build_track(events):
    events.sort(key=lambda e: (e[0], e[1]))
    body = b''
    prev = 0
    for tick, _, data in events:
        body += vlq(tick - prev) + data
        prev = tick
    return chunk(b'MTrk', body + b'\x00\xff\x2f\x00')


def write_midi(path, s):
    tempo = int(round(60000000.0 / s.bpm))
    name = s.name.encode('ascii', 'replace')
    meta = [(0, 0, b'\xff\x03' + vlq(len(name)) + name),
            (0, 1, b'\xff\x51\x03' + struct.pack('>I', tempo)[1:]),
            (0, 2, b'\xff\x58\x04\x04\x02\x18\x08')]

    channels = {}
    for tick, ch, pitch, vel, length in s.notes:
        channels.setdefault(ch, []).append(('n', tick, pitch, vel, length))
    for tick, ch, num, value in s.ccs:
        channels.setdefault(ch, []).append(('c', tick, num, value, 0))

    tracks = [build_track(meta)]
    for ch in sorted(channels):
        ev = []
        if ch in s.programs:
            ev.append((0, 0, bytes([0xC0 | ch, s.programs[ch]])))
        ev.append((0, 1, bytes([0xB0 | ch, 7, 127])))                      # громкость
        ev.append((0, 1, bytes([0xB0 | ch, 91, 110 if ch != 9 else 60])))  # реверб

        for item in channels[ch]:
            if item[0] == 'c':
                _, tick, num, value, _ = item
                ev.append((tick, 1, bytes([0xB0 | ch, num, value])))
            else:
                _, tick, pitch, vel, length = item
                ev.append((tick, 2, bytes([0x90 | ch, pitch, vel])))
                ev.append((tick + length, 3, bytes([0x80 | ch, pitch, 0])))

        tracks.append(build_track(ev))

    data = chunk(b'MThd', struct.pack('>HHH', 1, len(tracks), TPQ)) + b''.join(tracks)
    with open(path, 'wb') as f:
        f.write(data)
    return data


# --------------------------------------------------------------- WAV-превью

SR = 16000
TARGET_RMS = 0.20        # целевая средняя громкость превью


def render_wav(path, s):
    spb = 60.0 / s.bpm / TPQ
    total = int((s.seconds() + 4.0) * SR)
    buf = array.array('d', [0.0]) * total

    # кривые экспрессии по каналам (шаг 20 мс)
    grid = int(total / (SR * 0.02)) + 2
    expr = {}
    for ch in set(c[1] for c in s.ccs):
        pts = sorted((t * spb, v) for t, c, n, v in s.ccs if c == ch and n == 11)
        if not pts:
            continue
        curve = array.array('d', [1.0]) * grid
        for i in range(grid):
            t = i * 0.02
            v = pts[0][1]
            for pt, val in pts:
                if pt <= t:
                    v = val
                else:
                    break
            curve[i] = v / 100.0
        expr[ch] = curve

    for tick, ch, pitch, vel, length in s.notes:
        start = int(tick * spb * SR)
        dur = length * spb
        amp = vel / 127.0
        kind = s.voices.get(ch, 'sine')

        if ch == 9:
            render_drum(buf, start, pitch, amp)
            continue

        freq = 440.0 * (2.0 ** ((pitch - 69) / 12.0))
        render_voice(buf, start, dur, freq, amp, kind, expr.get(ch))

    # нормализация: тянем не пик, а среднюю громкость - иначе тихий эмбиент
    # с редкими всплесками звучит "как будто ничего нет"
    peak = 0.0
    energy = 0.0
    for v in buf:
        av = v if v > 0 else -v
        if av > peak:
            peak = av
        energy += v * v
    rms = math.sqrt(energy / total) if total else 0.0

    gain = 1.0
    if rms > 0:
        gain = TARGET_RMS / rms
    if peak > 0:
        gain = min(gain, 2.6 / peak)     # запас: пики дожимает tanh

    pcm = array.array('h', [0]) * total
    for i in range(total):
        pcm[i] = int(math.tanh(buf[i] * gain) * 32000)

    raw = pcm.tobytes()
    hdr = (b'RIFF' + struct.pack('<I', 36 + len(raw)) + b'WAVEfmt ' +
           struct.pack('<IHHIIHH', 16, 1, 1, SR, SR * 2, 2, 16) +
           b'data' + struct.pack('<I', len(raw)))
    with open(path, 'wb') as f:
        f.write(hdr + raw)


VOICE = {
    #          хвост  атака  спад   громкость
    'pad':    (0.6,   0.45,  0.60,  0.16),
    'bell':   (1.1,   0.002, 0.0,   0.28),
    'bass':   (0.15,  0.02,  0.25,  0.50),
    'sine':   (0.3,   0.002, 0.30,  0.20),
    'sub':    (1.5,   1.20,  2.50,  0.34),
    'bow':    (2.0,   2.20,  3.00,  0.50),
    'toll':   (5.0,   0.004, 0.0,   0.62),
    'choir':  (1.5,   1.00,  1.60,  0.46),
    'metal':  (2.0,   1.80,  2.40,  0.30),
}


def render_voice(buf, start, dur, freq, amp, kind, curve):
    tail, attack, release, level = VOICE.get(kind, VOICE['sine'])
    n = int((dur + tail) * SR)
    if start + n > len(buf):
        n = len(buf) - start
    if n <= 0:
        return

    decaying = kind in ('bell', 'toll')
    a = max(1, int(attack * SR))
    r = max(1, int(release * SR))
    step = 2.0 * math.pi * freq / SR
    det = 2.0 * math.pi * freq * 1.006 / SR
    det2 = 2.0 * math.pi * freq * 0.994 / SR
    lp = 0.0
    two_pi = 2.0 * math.pi

    for i in range(n):
        if i < a:
            env = i / float(a)
        elif decaying:
            env = math.exp(-3.5 * (i - a) / float(n - a + 1))
        else:
            tail_n = n - i
            env = 1.0 if tail_n > r else tail_n / float(r)
        if env <= 0.0:
            continue

        if curve is not None:
            env *= curve[min(len(curve) - 1, int((start + i) / (SR * 0.02)))]

        ph = i * step
        if kind in ('bow', 'metal', 'pad'):
            x1 = (i * step) % two_pi / math.pi - 1.0
            x2 = (i * det) % two_pi / math.pi - 1.0
            x3 = (i * det2) % two_pi / math.pi - 1.0
            y = (x1 + x2 + x3) / 3.0
            if kind == 'metal':
                y = y * (0.6 + 0.4 * math.sin(ph * 2.01))
            lp += 0.18 * (y - lp)
            y = lp
        elif kind == 'sub':
            y = math.sin(ph) * 0.8 + 0.2 * ((i * det) % two_pi / math.pi - 1.0)
            lp += 0.10 * (y - lp)
            y = lp
        elif kind == 'choir':
            y = (math.sin(ph) + 0.4 * math.sin(2 * ph) +
                 0.18 * math.sin(3 * ph) + 0.1 * math.sin(ph * 1.004)) * 0.6
        elif kind in ('bell', 'toll'):
            y = (math.sin(ph) + 0.35 * math.sin(2.0 * ph) +
                 0.12 * math.sin(3.01 * ph)) * 0.7
        elif kind == 'bass':
            y = ((i * step) % two_pi / math.pi - 1.0)
            lp += 0.25 * (y - lp)
            y = lp
        else:
            y = math.sin(ph)

        buf[start + i] += y * env * amp * level


def render_drum(buf, start, pitch, amp):
    if pitch in (36, 41):
        n = int(0.5 * SR)
        for i in range(n):
            if start + i >= len(buf):
                break
            env = math.exp(-9.0 * i / n)
            f = 70.0 * math.exp(-4.0 * i / n) + 32.0
            buf[start + i] += math.sin(2.0 * math.pi * f * i / SR) * env * amp * 0.9
    else:
        n = int(0.05 * SR)
        seed = 12345 + pitch
        for i in range(n):
            if start + i >= len(buf):
                break
            seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
            env = math.exp(-40.0 * i / n)
            buf[start + i] += ((seed / 0x3FFFFFFF) - 1.0) * env * amp * 0.25


# --------------------------------------------------------------------- main

STYLES = {'aperture': (score_aperture, 'assets/music/aperture_ambient.mid'),
          'stalker': (score_stalker, 'res/music.mid')}


def main():
    argv = sys.argv[1:]
    style = None
    preview = None

    if '--style' in argv:
        style = argv[argv.index('--style') + 1]
    if '--preview' in argv:
        preview = argv[argv.index('--preview') + 1]

    todo = [style] if style else ['aperture', 'stalker']

    for name in todo:
        maker, path = STYLES[name]
        s = maker()
        data = write_midi(path, s)
        print('%-9s -> %-32s %5d байт, %3d нот, %5.1f с, %d BPM' %
              (name, path, len(data), len(s.notes), s.seconds(), s.bpm))

        if preview and (style or name == 'stalker'):
            render_wav(preview, s)
            print('            превью: ' + preview)


if __name__ == '__main__':
    main()
