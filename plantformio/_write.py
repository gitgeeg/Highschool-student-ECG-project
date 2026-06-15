
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ECG Plotter - PyQtGraph, system-time based"""
import sys, argparse, traceback, threading, queue, math, copy, time
from collections import deque
import serial, serial.tools.list_ports, numpy as np

IMPORT_ERR = None
try:
    import pyqtgraph as pg
    from PyQt5 import QtCore, QtWidgets
    pg.setConfigOptions(antialias=False, useOpenGL=True)
except ImportError as e:
    IMPORT_ERR = str(e)

BAUD = 921600; WIN_RESP = 10; WIN_ECG = 5
MAXN_RESP = 40000; MAXN_ECG = 40000
MIN_STABLE = 12000  # 启动时忽略前3s数据（保持为0）

class NotchStage:
    def __init__(self, freq_hz, bw_hz=0, fs=4000):
        c = math.cos(2*math.pi*freq_hz/fs)
        r = max(.0, 1-bw_hz*math.pi/fs) if bw_hz>0 else .92
        self.b = [1, -2*c, 1]; self.a = [1, -2*r*c, r*r]
        self.w = [.0, .0, .0]
    def reset(self): self.w[:] = [.0, .0, .0]
    def filter(self, x):
        w0 = x - self.a[1]*self.w[1] - self.a[2]*self.w[2]
        y = self.b[0]*w0 + self.b[1]*self.w[1] + self.b[2]*self.w[2]
        self.w[2], self.w[1] = self.w[1], w0; return y

class Lowpass2:
    """2阶 Butterworth 低通 2Hz, fs=4000 (呼吸用)"""
    def __init__(self, fs=4000):
        wc = 2.0 / (fs / 2)
        Wc = math.tan(math.pi * wc / 2)
        d = 1 + math.sqrt(2)*Wc + Wc**2
        self.b = [Wc**2/d, 2*Wc**2/d, Wc**2/d]
        self.a = [1, 2*(Wc**2-1)/d, (1-math.sqrt(2)*Wc+Wc**2)/d]
        self.w = [.0, .0, .0]
    def reset(self): self.w[:] = [.0, .0, .0]
    def filter(self, x):
        w0 = x - self.a[1]*self.w[1] - self.a[2]*self.w[2]
        y = self.b[0]*w0 + self.b[1]*self.w[1] + self.b[2]*self.w[2]
        self.w[2], self.w[1] = self.w[1], w0; return y

class Lowpass25:
    """2阶 Butterworth 低通 25Hz, fs=4000"""
    def __init__(self, fs=4000):
        wc = 25.0 / (fs / 2)
        Wc = math.tan(math.pi * wc / 2)
        d = 1 + math.sqrt(2)*Wc + Wc**2
        self.b = [Wc**2/d, 2*Wc**2/d, Wc**2/d]
        self.a = [1, 2*(Wc**2-1)/d, (1-math.sqrt(2)*Wc+Wc**2)/d]
        self.w = [.0, .0, .0]
    def reset(self): self.w[:] = [.0, .0, .0]
    def filter(self, x):
        w0 = x - self.a[1]*self.w[1] - self.a[2]*self.w[2]
        y = self.b[0]*w0 + self.b[1]*self.w[1] + self.b[2]*self.w[2]
        self.w[2], self.w[1] = self.w[1], w0; return y

class DCBlocker:
    """1阶 DC blocker，去除基线漂移"""
    def __init__(self, R=0.995):
        self.R = R; self.x1 = .0; self.y1 = .0
    def reset(self): self.x1 = .0; self.y1 = .0
    def filter(self, x):
        y = x - self.x1 + self.R * self.y1
        self.x1, self.y1 = x, y; return y

class RespFilterChain:
    """DC blocker(0.32Hz) + 2Hz低通"""
    def __init__(self):
        self.dc = DCBlocker(0.9995)
        self.lp2 = Lowpass2()
    def reset(self): self.dc.reset(); self.lp2.reset()
    def filter(self, x): return self.lp2.filter(self.dc.filter(x))

class FilterChain:
    """DC blocker(3.2Hz) + 48Hz陷波 + 100Hz陷波 + 25Hz低通"""
    def __init__(self):
        self.dc = DCBlocker(0.995)
        self.n48 = NotchStage(48, 10)
        self.n100 = NotchStage(100, 5)
        self.lp25 = Lowpass25()
    def reset(self): self.dc.reset(); self.n48.reset(); self.n100.reset(); self.lp25.reset()
    def filter(self, x): return self.lp25.filter(self.n100.filter(self.n48.filter(self.dc.filter(x))))

class Reader:
    def __init__(self, port, baud, dst):
        self.dst = dst; self.ser = None
        try: self.ser = serial.Serial(port, baud, timeout=1.0)
        except Exception as e: raise SystemExit('[err] '+str(e))
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        print('[connected]', port, '@', baud)
    def _loop(self):
        while self._run:
            try:
                raw = self.ser.readline()
                if not raw: continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line or len(line) < 18: continue
                clean = ''.join(ch for ch in line if ch in '0123456789abcdefABCDEF')
                if len(clean) >= 18: self.dst.put_nowait((time.perf_counter(), clean[:18]))
            except serial.SerialException: self._run = False
            except Exception: traceback.print_exc(); self._run = False
    def close(self):
        self._run = False
        if self.ser and self.ser.is_open: self.ser.close()

class Buffer:
    def __init__(self, flt2=None, flt1=None):
        self.flt2 = flt2
        self.flt2_inst = copy.deepcopy(flt2) if flt2 else None
        self.flt1 = flt1
        self.flt1_inst = copy.deepcopy(flt1) if flt1 else None
        self._bypass = False  # 默认开启50Hz滤波
        self.c1 = deque(maxlen=MAXN_RESP); self.c2 = deque(maxlen=MAXN_ECG)
        self.f1 = deque(maxlen=MAXN_RESP); self.f2 = deque(maxlen=MAXN_ECG)
        self.lo1 = deque(maxlen=MAXN_RESP); self.lo2 = deque(maxlen=MAXN_ECG)
        self.lor = deque(maxlen=MAXN_RESP)
        self.tm1 = deque(maxlen=MAXN_RESP); self.tm2 = deque(maxlen=MAXN_ECG)
        self.n = self.err = 0
        self._t0 = time.perf_counter()  # 系统时钟基准
        self.latest_gpio = 0; self.latest_status = '0x000000'
    def toggle_filter(self):
        if not self.flt2: return False
        self._bypass = not self._bypass
        # f1始终走5Hz低通（独立于CH2滤波开关）
        if self.flt1_inst:
            self.flt1_inst.reset()
            self.f1.clear()
            for v in self.c1: self.f1.append(self.flt1_inst.filter(v))
        else:
            self.f1.clear()
            for v in self.c1: self.f1.append(v)
        # f2取决于bypass状态
        if self._bypass:
            self.f2.clear()
            for v in self.c2: self.f2.append(v)
        else:
            if self.flt2_inst: self.flt2_inst.reset()
            self.f2.clear()
            for v in self.c2:
                x = v
                if self.flt2_inst: x = self.flt2_inst.filter(x)
                self.f2.append(x)
        return not self._bypass
    @property
    def filter_active(self): return self.flt2 is not None and not self._bypass
    def add(self, line, ts=None):
        try:
            raw = bytes.fromhex(line[:18])
            if len(raw) < 9: return False
            status = (raw[0]<<16)|(raw[1]<<8)|raw[2]
            lo1 = ((status>>20)&1 or (status>>19)&1) != 0
            lo2 = ((status>>22)&1 or (status>>21)&1) != 0
            lor = ((status>>23)&1) != 0
            ch1 = (raw[3]<<16)|(raw[4]<<8)|raw[5]
            if ch1 & 0x800000: ch1 |= ~0xFFFFFF
            ch2 = (raw[6]<<16)|(raw[7]<<8)|raw[8]
            if ch2 & 0x800000: ch2 |= ~0xFFFFFF
            now = (ts if ts else time.perf_counter()) - self._t0  # 串口接收时间戳
            self.c1.append(float(ch1)); self.c2.append(float(ch2))
            v1 = float(ch1)
            if self.flt1_inst: v1 = self.flt1_inst.filter(v1)
            self.f1.append(v1)
            v = float(ch2)
            if not self._bypass and self.flt2_inst: v = self.flt2_inst.filter(v)
            self.f2.append(v)
            self.lo1.append(lo1); self.lo2.append(lo2); self.lor.append(lor)
            self.tm1.append(now); self.tm2.append(now)
            gpio2 = (status>>17)&1; gpio1 = (status>>16)&1
            self.latest_gpio = (gpio2<<1)|gpio1
            self.latest_status = '0x{:06X}'.format(status)
            self.n += 1; return True
        except: self.err += 1; return False
    @property
    def len(self): return len(self.c1)
    def get_data(self):
        return (np.array(self.tm1), np.array(self.tm2),
                np.array(self.f1), np.array(self.f2),
                bool(self.lo1[-1]) if self.lo1 else False,
                bool(self.lo2[-1]) if self.lo2 else False,
                bool(self.lor[-1]) if self.lor else False)

def find_port():
    ports = serial.tools.list_ports.comports()
    for p in sorted(ports):
        d = (p.description or '').lower()
        if any(k in d for k in ('ch340','cp210','arduino','uart')): return p.device
    return sorted(ports)[0].device if ports else None

class ECGApp:
    def __init__(self, port, flt2, flt1=None):
        self.buf = Buffer(flt2=flt2, flt1=flt1)
        self.q = queue.Queue()
        self.reader = Reader(port, BAUD, self.q)
        self.win = pg.GraphicsLayoutWidget(title='ECG -- 24-bit')
        self.win.setMinimumWidth(950)
        self.p1 = self.win.addPlot(title='CH1 (10s)')
        self.p1.setLabel('left', 'CH1')
        self.p1.showGrid(x=True,y=True,alpha=.2)
        self.c1_avg_curve = self.p1.plot(pen=pg.mkPen('#e67e22',width=2))
        self.win.nextRow()
        self.p2 = self.win.addPlot(title='CH2 ECG (5s)')
        self.p2.setLabel('left','CH2'); self.p2.setLabel('bottom','Time (s)')
        self.p2.showGrid(x=True,y=True,alpha=.2)
        self.c2_curve = self.p2.plot(pen=pg.mkPen('#2980b9',width=1))
        self.p2.setYRange(-6000, 6000)  # 固定Y轴范围，不清除缩放
        self._ecg_base = 0.0  # ECG运行基准跟踪
        self.btn = QtWidgets.QPushButton('Filter:ON')
        self.btn.setCheckable(True); self.btn.setChecked(True)
        self.btn.setStyleSheet('QPushButton{background:#d5f0d5;font:10pt}')
        self.btn.toggled.connect(self._on_filter_toggle)
        self._sweep_t0 = 0.0  # 当前扫描起始时间
        # 上次有效统计值（无新数据时保持显示）
        self._resp_rate = '**'; self._resp_period = '**'
        self._ecg_hr = '**'; self._ecg_rr = '**'
        self._ecg_sdnn = '**'; self._ecg_rmssd = '**'; self._ecg_last5 = '**'
        self._total_r_peaks = 0  # 累计R波总数（只增不减）
        self._last_abs_pk = -1   # 上次最后一个峰的绝对序号
        # 右侧信息面板（1/4宽度）- 呼吸在上(黄) 心电在下(绿)
        self.resp_box = QtWidgets.QTextEdit()
        self.resp_box.setReadOnly(True)
        self.resp_box.setStyleSheet('QTextEdit{background:#3d2b00;color:#fff;font:13pt Consolas;padding:10px}')
        self.ecg_box = QtWidgets.QTextEdit()
        self.ecg_box.setReadOnly(True)
        self.ecg_box.setStyleSheet('QTextEdit{background:#003300;color:#fff;font:13pt Consolas;padding:10px}')
        self.right_panel = QtWidgets.QWidget()
        vlay = QtWidgets.QVBoxLayout()
        vlay.setContentsMargins(0, 0, 0, 0)
        vlay.setSpacing(0)
        vlay.addWidget(self.resp_box, 1)
        vlay.addWidget(self.ecg_box, 1)
        self.right_panel.setLayout(vlay)
        self.container = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.win, 3)
        layout.addWidget(self.right_panel, 1)
        self.container.setLayout(layout)
        self.container.setWindowTitle('ECG -- 24-bit')
        self.container.resize(1300, 650)
        self.container.show()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(40)
    def _on_filter_toggle(self, chk):
        on = self.buf.toggle_filter()
        self.btn.setText('Filter:ON' if on else 'Filter:OFF')
        s = '#d5f0d5' if on else '#f0d5d5'
        self.btn.setStyleSheet('QPushButton{{background:{};font:10pt}}'.format(s))
    def _update(self):
        while not self.q.empty():
            try:
                ts, line = self.q.get_nowait(); self.buf.add(line, ts)
            except queue.Empty: break
        if self.buf.len < MIN_STABLE: return  # 等待数据稳定再绘图
        t1, t2, c1, c2, lo1, lo2, lor = self.buf.get_data()
        ds = 8  # 4kHz → 500Hz 降采样
        if len(t1) > 0:
            self.p1.setXRange(t1[-1]-WIN_RESP, max(t1[-1], WIN_RESP))
            # 呼吸曲线：0.2s 滑动平均 + 降采样
            win_s = int(0.2 / 0.00025)  # 0.2秒 = 800点
            if len(c1) >= win_s:
                kernel = np.ones(win_s) / win_s
                c1_avg = np.convolve(c1, kernel, mode='same')
                self.c1_avg_curve.setData(t1[::ds], c1_avg[::ds])
        if len(t2) > 0:
            # ECG：固定窗口，从左到右扫描刷新（像心电图机一样）
            last_t = t2[-1]
            if last_t - self._sweep_t0 >= WIN_ECG:
                self._sweep_t0 = last_t  # 开始新扫描
            # 只显示当前扫描窗口内的数据
            mask = (t2 >= self._sweep_t0) & (t2 < self._sweep_t0 + WIN_ECG)
            if np.any(mask):
                # 块平均降采样
                t_sw = t2[mask]
                c_sw = c2[mask]
                n_ds = len(c_sw) // ds
                if n_ds > 0:
                    c2_ds = np.mean(c_sw[:n_ds*ds].reshape(-1, ds), axis=1)
                    t2_ds = t_sw[ds-1::ds][:n_ds] - self._sweep_t0
                    # 运行基准跟踪（每帧更新1%，扫描切换时无缝衔接）
                    if len(c2_ds) > 0:
                        cur_base = np.percentile(c2_ds, 10)
                        self._ecg_base = self._ecg_base * 0.99 + cur_base * 0.01
                    self.p2.setXRange(0, WIN_ECG)
                    self.c2_curve.setData(t2_ds, c2_ds - self._ecg_base)
            color = '#27ae60' if self.buf.filter_active else '#2980b9'
            self.c2_curve.setPen(pg.mkPen(color, width=1))
        # 导联脱落指示已关闭
        # --- 呼吸(上) + 心电(下) 右侧面板 ---
        # 呼吸分析 - 动态跳过伪迹区域，有新数据才覆盖
        if len(c1) >= 6000:
            seg = c1[-min(len(c1), 40000):]
            # 找到最后一个超幅值之后的位置作为稳定起点
            bad = np.where(np.abs(seg) > 50000)[0]
            start = bad[-1] + 1 if len(bad) > 0 else 0
            seg = seg[start:]
            if len(seg) >= 2000:
                seg_z = seg - np.mean(seg)
                zc = np.where((seg_z[:-1] <= 0) & (seg_z[1:] > 0))[0]
                if len(zc) >= 3:
                    resp_int = np.diff(zc) * 0.00025
                    valid = resp_int[resp_int <= 10]
                    if len(valid) >= 1:
                        last_rp = valid[-1]
                        self._resp_rate = '{:3.0f} bpm'.format(60.0 / last_rp)
                        self._resp_period = '{:4.2f} s'.format(last_rp)
        self.resp_box.setHtml(
            '<html><body style="font-family:Consolas;font-size:14pt;color:#fff;text-align:center">'
            '<p style="font-size:20pt;font-weight:bold;color:#ffb74d;margin:10px 0;text-align:center">══ Resp ══</p>'
            '<p style="font-size:26pt;font-weight:bold;color:#fff;margin:8px 0;text-align:center">' + self._resp_rate + '</p>'
            '<p style="font-size:16pt;color:#ccc;margin:4px 0;text-align:center">' + self._resp_period + '</p>'
            '</body></html>')
        # ECG分析 - 有新数据才覆盖，无则保持上次值
        try:
            if len(c2) >= 4000:
                sig = c2[:]
                # 自适应阈值：99百分位数×0.48（抗单点伪迹，百分位不受个别尖峰影响）
                th = np.clip(np.percentile(sig, 99) * 0.48, 1800, 4000)
                min_dist = 1200
                pk = []
                last = -min_dist
                for i in range(min_dist, len(sig)-min_dist):
                    if sig[i] > th and sig[i] == np.max(sig[i-min_dist:i+min_dist+1]):
                        if i - last >= min_dist:
                            pk.append(i)
                            last = i
                # 累计R波计数：统计所有比上次新的峰
                abs_base = self.buf.n - len(sig)
                if len(pk) > 0:
                    new_pks = sum(1 for p in pk if abs_base + p > self._last_abs_pk)
                    self._total_r_peaks += new_pks
                    self._last_abs_pk = abs_base + pk[-1]
                # RR间期：用系统时间戳计算（比样本计数更准）
                if len(pk) >= 2:
                    last_rr = (t2[pk[-1]] - t2[pk[-2]]) * 1000  # 秒→毫秒
                    self._ecg_hr = '{:3.0f} bpm'.format(60000.0 / last_rr)
                    self._ecg_rr = '{:3.1f} ms'.format(last_rr)
                # HRV：用系统时间戳计算
                if len(pk) >= 4:
                    intervals = np.diff(t2[pk]) * 1000
                    valid = intervals[(intervals >= 300) & (intervals <= 2000)]
                    if len(valid) >= 3:
                        self._ecg_sdnn = '{:3.0f}ms'.format(np.std(valid, ddof=1))
                        self._ecg_rmssd = '{:3.0f}ms'.format(np.sqrt(np.mean(np.diff(valid)**2)))
                    if len(valid) >= 1:
                        self._ecg_last5 = '  '.join('{:3.1f}'.format(x) for x in valid[-5:])
        except Exception as e:
            print('[ECG err]', e)
        self.ecg_box.setHtml(
            '<html><body style="font-family:Consolas;font-size:14pt;color:#fff;text-align:center">'
            '<p style="font-size:20pt;font-weight:bold;color:#4fc3f7;margin:10px 0;text-align:center">══ ECG ══</p>'
            '<p style="font-size:26pt;font-weight:bold;color:#fff;margin:8px 0;text-align:center">' + self._ecg_hr + '</p>'
            '<p style="font-size:16pt;color:#ccc;margin:4px 0;text-align:center">RR: ' + self._ecg_rr + '</p>'
            '<p style="font-size:14pt;color:#aaa;margin:4px 0;text-align:center">SDNN:' + self._ecg_sdnn + ' RMSSD:' + self._ecg_rmssd + '</p>'
            '<p style="font-size:14pt;color:#888;margin:4px 0;text-align:center">' + self._ecg_last5 + '</p>'
            '<p style="font-size:13pt;color:#666;margin:2px 0;text-align:center">R peaks: ' + str(self._total_r_peaks) + '</p>'
            '</body></html>')
    def close(self): self.timer.stop(); self.reader.close()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p','--port'); ap.add_argument('--list',action='store_true')
    ap.add_argument('--filter-mode',choices=['on','off'],default='on')
    args = ap.parse_args()
    if IMPORT_ERR:
        print('[err]', IMPORT_ERR); print('Install: pip install pyqtgraph PyQt5'); sys.exit(1)
    if args.list:
        for p in sorted(serial.tools.list_ports.comports()): print(' ', p.device, '-', p.description)
        return
    port = args.port or find_port()
    if not port: print('[err] no serial port found'); sys.exit(1)
    flt1 = RespFilterChain()
    flt2 = FilterChain() if args.filter_mode != 'off' else None
    if flt2: print('[filter] DC block + 48Hz+100Hz Notch + 25Hz Lowpass on CH2')
    print('[filter] DC block + 2Hz Lowpass on CH1 (respiration)')
    app = QtWidgets.QApplication(sys.argv)
    ecg = ECGApp(port, flt2, flt1=flt1)
    try: app.exec_()
    except KeyboardInterrupt: pass
    finally: ecg.close()

if __name__ == '__main__': main()
