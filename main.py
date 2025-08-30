# -*- coding: utf-8 -*-
import cv2, numpy as np
import time, math, re, threading
import os, csv
from pathlib import Path
import pandas as pd
import pickle

# ====== 포트/카메라 ======
SERIAL_PORT = 'COM12'     # 환경에 맞게
SERIAL_BAUD = 115200
CAM_INDEX   = 0

# ====== 아두이노 명령(스케치와 일치) ======
CMD_FWD    = 'I'   # Forward
CMD_REV    = 'O'   # Reverse
CMD_STOP   = 'S'
STEP_ONCE  = 'n'   # 소문자/대문자 모두 허용됨(스케치가 대문자로 변환)

# ====== 비전 파라미터 ======
AREA_MIN = 40
KERNEL   = np.ones((5,5), np.uint8)
RED1_L, RED1_H = (0, 50, 50),   (15,255,255)
RED2_L, RED2_H = (160,50,50),   (180,255,255)
BLUE_L , BLUE_H= (95,120,80),   (135,255,255)

# 시계판 정규화 기준(호모그래피 목적지)
OUT_W, OUT_H = 480, 480
CENTER = np.array([OUT_W//2, OUT_H//2], np.float32)
RADIUS = 170
Q = np.float32([
    CENTER + RADIUS*np.array([ 0.0,-1.0], np.float32),  # 12시 RED
    CENTER + RADIUS*np.array([ 1.0, 0.0], np.float32),  # 3시 BLUE
    CENTER + RADIUS*np.array([ 0.0, 1.0], np.float32),  # 6시 BLUE
    CENTER + RADIUS*np.array([-1.0, 0.0], np.float32),  # 9시 BLUE
])

pi = 3.14

# ====== 세션 파라미터 ======
ROTATIONS_TARGET   = 10
TRG_DEG, TOL_DEG   = -90.0, 5.0
HYST_DEG           = 8.0
STALL_OMEGA_THRESH = 7.0        # deg/s
STALL_DWELL_SEC    = 2.0
SESSION_TIMEOUT_S  = 150.0
AUTO_RET_PAUSE_SEC = 2.0
FINAL_PUSH_SEC     = 0.6

# stop-at-position
USE_STOP_ANGLE   = True
STOP_THETA_DEG   = -90.0
STOP_TOL_DEG     = 2.0
STOP_PT          = (float(CENTER[0]), float(CENTER[1]-RADIUS))
STOP_TOL_PX      = 6.0
STOP_MAX_SPEED_DEG_S = None

# ====== 플롯 ======
PLOT_W, PLOT_H, MARGIN = 700, 260, 36
STRIP_LEN = 300

def init_plot():
    img = np.zeros((PLOT_H, PLOT_W, 3), np.uint8); img[:] = (20,20,20); return img

def draw_axes(img, top, bottom, title, y_min, y_max):
    cv2.rectangle(img, (MARGIN, top), (PLOT_W-MARGIN, bottom), (80,80,80), 1, cv2.LINE_AA)
    cv2.putText(img, title, (MARGIN+4, top-8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1, cv2.LINE_AA)
    if y_min < 0 < y_max:
        y0 = int(np.interp(0, [y_min, y_max], [bottom, top]))
        cv2.line(img, (MARGIN, y0), (PLOT_W-MARGIN, y0), (60,60,90), 1, cv2.LINE_AA)

def plot_series(img, top, bottom, vals, color, y_min, y_max):
    if len(vals) < 2: return
    xs = np.linspace(MARGIN, PLOT_W-MARGIN, num=len(vals))
    ys = np.interp(vals, [y_min, y_max], [bottom, top])
    pts = np.vstack([xs, ys]).T.astype(np.int32)
    for i in range(1, len(pts)):
        cv2.line(img, tuple(pts[i-1]), tuple(pts[i]), color, 1, cv2.LINE_AA)

# ====== 유틸 ======
def wrap_to_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
def rad2deg(r):    return r * 180.0 / math.pi

def mask_hsv(hsv, lo, hi):
    return cv2.inRange(hsv, np.array(lo,np.uint8), np.array(hi,np.uint8))

def find_centroids(mask, max_pts, area_min):
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pts, areas = [], []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < area_min: continue
        M = cv2.moments(c)
        if M['m00']==0: continue
        pts.append((M['m10']/M['m00'], M['m01']/M['m00']))
        areas.append(a)
    if not pts: return np.empty((0,2), np.float32)
    idx = np.argsort(areas)[::-1][:max_pts]
    return np.float32([pts[i] for i in idx])

def angle_from_center(p, c): return math.atan2(p[1]-c[1], p[0]-c[0])

def build_H_from_points(reds, blues):
    C = (reds[0] + blues.mean(axis=0)) / 2.0
    def norm(a): return (a + 2*math.pi) % (2*math.pi)   # ← math.pi 로 수정
    r_ang = norm(angle_from_center(reds[0], C))
    b_ang = [norm(angle_from_center(b, C)) for b in blues]
    diffs = [(i, (b_ang[i] - r_ang) % (2*math.pi)) for i in range(3)]  # ← math.pi 로 수정
    diffs.sort(key=lambda x: x[1])
    P = np.float32([reds[0], blues[diffs[0][0]], blues[diffs[1][0]], blues[diffs[2][0]]])
    H_new, _ = cv2.findHomography(P, Q, method=0)
    return H_new


def is_at_stop_by_angle(theta_deg, stop_deg, tol_deg):
    d = abs((theta_deg - stop_deg + 180.0) % 360.0 - 180.0)
    return d <= tol_deg

def is_at_stop_by_xy(pix, stop_pt, tol_px):
    dx = float(pix[0] - stop_pt[0]); dy = float(pix[1] - stop_pt[1])
    return (dx*dx + dy*dy) <= (tol_px*tol_px)

# ====== 시리얼 ======
try:
    import serial; _SERIAL_OK=True
except Exception as e:
    print("[WARN] pyserial missing:", e); _SERIAL_OK=False

class SerialIO:
    NUMERIC_RE = re.compile(r'^\s*(-?\d+)\s*$')
    def __init__(self, port, baud):
        self.port=port; self.baud=baud
        self.ser=None; self.stop=threading.Event()
        self.thread=None; self.lock=threading.Lock()
        self.latest_pot=None; self._event_once=None; self.tx_last=''
    def open(self):
        if not _SERIAL_OK: return False
        try:
            self.ser=serial.Serial(self.port,self.baud,timeout=0.05)
            self.thread=threading.Thread(target=self._rx_loop,daemon=True); self.thread.start()
            print(f"[SER] open {self.port}@{self.baud}"); return True
        except Exception as e:
            print("[SER] open fail:", e); return False
    def _rx_loop(self):
        buf=b''
        while not self.stop.is_set() and self.ser:
            try:
                chunk=self.ser.read(128)
                if not chunk: continue
                buf+=chunk
                while b'\n' in buf:
                    line,buf=buf.split(b'\n',1)
                    s=line.decode(errors='ignore').strip()
                    m=self.NUMERIC_RE.match(s)
                    if m:
                        try:
                            v=int(m.group(1))
                            with self.lock: self.latest_pot=v
                        except: pass
                    else:
                        with self.lock: self._event_once=s
            except Exception:
                time.sleep(0.02)
    def pop_event(self):
        with self.lock:
            ev=self._event_once; self._event_once=None; return ev
    def get_pot(self):
        with self.lock: return self.latest_pot
    def send_line(self, msg:str):
        if not msg: return
        if self.ser and self.ser.is_open:
            if not msg.endswith('\n'): msg+='\n'
            try:
                self.ser.write(msg.encode('ascii')); self.tx_last=msg.strip()
            except Exception: pass
    def send_char(self, ch:str): self.send_line(ch)
    def close(self):
        self.stop.set()
        if self.thread: self.thread.join(0.5)
        if self.ser:
            try: self.ser.close()
            except: pass

# ====== (내장) 분석/예측 ======
LOGS_DIR = Path("logs")
MODEL_PATHS = [LOGS_DIR/"xgb_model.pkl", Path("xgb_model.pkl")]

EVENT_FILTER = ["TX:I", "TX:T415", "TX:T475", "FAIL:STALL>2", "OK: rot=10", "TIMEOUT"]

def _normalize_event(ev: str) -> str:
    if not isinstance(ev, str): return ""
    s = ev.strip()
    if s == "TX:i": s = "TX:I"
    if "FAIL_STALL>2" in s: s = "FAIL:STALL>2"
    if s == "OK: return_to_stop": s = "OK: rot=10"
    return s

def theta_shift(theta):
    try: th = float(theta)
    except: th = 0.0
    shifted = th + 90.0
    if shifted < -5.0: shifted += 360.0
    return shifted

def run_inline_analysis(latest_csv: Path):
    try:
        df0 = pd.read_csv(latest_csv)
    except Exception as e:
        print(f"[ANALYSIS] read fail: {e}")
        return None

    for c in ["time_sec","rpm","theta_deg","rot","stall","event"]:
        if c not in df0.columns: df0[c] = 0

    df0["event"] = df0["event"].astype(str).map(_normalize_event)

    try:
        df = (
            df0.loc[lambda d: d["event"].isin(EVENT_FILTER)]
               .loc[lambda d: d["rot"] != 0, ["time_sec","rpm","theta_deg","rot","stall","event"]]
        )
    except Exception as e:
        print(f"[ANALYSIS] preprocess fail: {e}")
        return None

    if df.empty:
        print("[ANALYSIS] skip: filtered empty")
        return None

    df["time_sec"] = pd.to_numeric(df["time_sec"], errors="coerce").fillna(method="ffill").fillna(0.0)
    df["rpm"]      = pd.to_numeric(df["rpm"],      errors="coerce").interpolate(method="linear", limit_direction="both")
    df["theta_deg"]= pd.to_numeric(df["theta_deg"],errors="coerce").fillna(0.0)
    df["rot"]      = pd.to_numeric(df["rot"],      errors="coerce").ffill().fillna(0).astype(int)
    df["stall"]    = pd.to_numeric(df["stall"],    errors="coerce").fillna(0).astype(int)

    df["time_sec"] = df["time_sec"] - df["time_sec"].min()
    final_rotation = int(df["rot"].iloc[-1])

    total_time  = float(df["time_sec"].max())
    mean_rpm    = float(df["rpm"].mean())
    stall_count = int(df["stall"].max())
    max_rpm     = float(df["rpm"].max())
    std_rpm     = float(df["rpm"].std())

    stall_times, stall_positions = [], []
    prev = None
    for t_sec, theta, rot, s in zip(df["time_sec"], df["theta_deg"], df["rot"], df["stall"]):
        if prev is None: prev = s; continue
        if s > prev:
            stall_times.append(round(float(t_sec),3))
            pos = (rot - 1) * 360 + theta_shift(theta)
            stall_positions.append(round(float(pos),3))
        prev = s
    while len(stall_times) < 2:      stall_times.append(0.0)
    while len(stall_positions) < 2:  stall_positions.append(0.0)

    theta_shifted = df["theta_deg"].apply(theta_shift)
    positions = (df["rot"] - 1) * 360 + theta_shifted
    total_rotation_deg = float(positions.iloc[-1] - positions.iloc[0])

    summary_row = {
        "Total_Time_sec": round(total_time, 2),
        "Avg_RPM": round(mean_rpm, 2),
        "Final_Rotation_rot": final_rotation,
        "Stall_Max_Value": stall_count,
        "Stall_Increase_Time1": stall_times[0],
        "Stall_Increase_Time2": stall_times[1],
        "Stall_Increase_Pos1": stall_positions[0],
        "Stall_Increase_Pos2": stall_positions[1],
        "Total_Rotation_deg": round(total_rotation_deg, 2),
        "Max_RPM": round(max_rpm, 2),
        "RPM_StdDev": round(std_rpm, 2),
    }

    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    pd.DataFrame([summary_row]).to_csv(LOGS_DIR/"summary.csv", index=False, encoding="utf-8-sig")

    # 모델 로드
    model = None
    for p in MODEL_PATHS:
        if p.exists():
            try:
                with open(p, "rb") as f: model = pickle.load(f)
                print(f"[ANALYSIS] model loaded: {p}")
                break
            except Exception as e:
                print(f"[ANALYSIS] model load fail ({p}): {e}")
    if model is None:
        print("[ANALYSIS] model not found → skip prediction")
        return None

    feature_cols = [
        "Total_Time_sec","Avg_RPM","Final_Rotation_rot",
        "Stall_Max_Value","Stall_Increase_Time1","Stall_Increase_Time2",
        "Stall_Increase_Pos1","Stall_Increase_Pos2",
        "Total_Rotation_deg","Max_RPM","RPM_StdDev"
    ]
    X = pd.DataFrame([summary_row]).reindex(columns=feature_cols).fillna(0)
    try:
        y_pred = model.predict(X)
        pred = int(y_pred[0])
        out_df = pd.DataFrame([summary_row]); out_df["Model_Prediction"] = pred
        out_df.to_csv(LOGS_DIR/"final_summary_with_prediction.csv", index=False, encoding="utf-8-sig")
        print(f"[ANALYSIS] final → {LOGS_DIR/'final_summary_with_prediction.csv'}")
        print(out_df)
        return pred
    except Exception as e:
        print(f"[ANALYSIS] predict fail: {e}")
        return None

# ====== 메인 ======
def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened(): raise RuntimeError("Cannot open camera")
    cv2.namedWindow("raw",   cv2.WINDOW_NORMAL)
    cv2.namedWindow("plots", cv2.WINDOW_NORMAL)

    sio = SerialIO(SERIAL_PORT, SERIAL_BAUD); sio.open()

    # ----- CSV -----
    os.makedirs("logs", exist_ok=True)
    session_id = time.strftime("session_%Y%m%d_%H%M%S")
    file_idx = 0
    csv_fp = None
    writer = None
    recording = False
    t0 = None

    pending_csv_rotate = False
    skip_log_this_frame = False

    def _open_csv(start_paused=True):
        nonlocal csv_fp, writer, file_idx, recording, t0
        if csv_fp:
            try: csv_fp.close()
            except: pass
        fname = Path("logs") / f"session_{session_id}_idx{file_idx:02d}.csv"
        file_idx += 1
        csv_fp = open(str(fname), "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_fp)
        writer.writerow(["time_sec","pot_raw","rpm","theta_deg","rot","stall","event"])
        csv_fp.flush()
        recording = not start_paused
        t0 = None
        print(f"[LOG] writing → {fname} (recording={'ON' if recording else 'PAUSED'})")

    def _write_next_marker(t_sec, pot_raw, rpm, theta_deg, rot_count, stall_count):
        nonlocal writer
        if writer is None: return
        writer.writerow([
            f"{(0.0 if t_sec is None else t_sec):.6f}",
            "" if pot_raw is None else int(pot_raw),
            "" if rpm is None else f"{rpm:.6f}",
            "" if theta_deg is None else f"{theta_deg:.2f}",
            rot_count if rot_count is not None else "",
            stall_count if stall_count is not None else "",
            "NEXT"
        ])
        csv_fp.flush()

    _open_csv(start_paused=True)

    print("=== Controls ===")
    print("i: start/resume | o: pause | O: manual return | s: EVENT=S | n: rotate CSV + analyze(+LED) | ESC: quit")

    # ----- 상태 변수 -----
    H=None
    prev_t=time.perf_counter()
    prev_theta=None; omega_s=None

    session_on=False; session_t0=None
    rot_count=0; stall_count=0; stall_timer=0.0; armed=True
    sent_T1=False; sent_T2=False

    MODE_FWD, MODE_RET = 0, 1
    RET_NONE, RET_AUTO, RET_MANUAL = 0, 1, 2
    mode = MODE_FWD
    ret_mode = RET_NONE
    buf_rpm=[]

    def reset_session(set_mode=MODE_FWD):
        nonlocal session_on, session_t0, rot_count, stall_count, stall_timer, armed
        nonlocal sent_T1, sent_T2, mode, omega_s, prev_theta, ret_mode
        session_on=True; session_t0=time.perf_counter()
        sent_T1=False; sent_T2=False
        mode = set_mode; ret_mode = RET_NONE
        omega_s=None; prev_theta=None
        armed=True; stall_timer=0.0
        if mode == MODE_FWD:
            rot_count=0; stall_count=0
            sio.send_char(CMD_FWD)
        else:
            sio.send_char(CMD_REV)

    def enter_ret_auto(reason="GENERIC"):
        nonlocal mode, ret_mode, armed, session_on
        if reason == "TARGET" and FINAL_PUSH_SEC and FINAL_PUSH_SEC > 0:
            time.sleep(FINAL_PUSH_SEC)
        sio.send_char(CMD_STOP)
        time.sleep(AUTO_RET_PAUSE_SEC)
        mode = MODE_RET; ret_mode = RET_AUTO
        session_on = True; armed = True
        sio.send_char(CMD_REV)

    def enter_ret_manual():
        nonlocal session_on, session_t0, mode, ret_mode, armed, stall_timer
        mode = MODE_RET; ret_mode = RET_MANUAL
        if not session_on:
            session_on=True; session_t0=time.perf_counter()
        armed = True; stall_timer = 0.0
        sio.send_char(CMD_REV)

    def end_session(_reason=""):
        nonlocal session_on, ret_mode
        sio.send_char(CMD_STOP)
        session_on=False; ret_mode = RET_NONE

    try:
        while True:
            ok, frame = cap.read()
            if not ok: break
            now = time.perf_counter()
            dt  = now - prev_t if now>prev_t else 0.0
            prev_t = now

            # 색 마스크
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_r = cv2.bitwise_or(mask_hsv(hsv, RED1_L, RED1_H), mask_hsv(hsv, RED2_L, RED2_H))
            mask_b = mask_hsv(hsv, BLUE_L, BLUE_H)
            mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN,  KERNEL)
            mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_CLOSE, KERNEL)
            mask_b = cv2.morphologyEx(mask_b, cv2.MORPH_OPEN,  KERNEL)
            mask_b = cv2.morphologyEx(mask_b, cv2.MORPH_CLOSE, KERNEL)

            raw = frame.copy()
            reds  = find_centroids(mask_r, max_pts=1, area_min=AREA_MIN)
            blues = find_centroids(mask_b, max_pts=3, area_min=AREA_MIN)

            # 키 입력
            event_override = ""
            key=cv2.waitKey(1)&0xFF
            if key==27: break
            elif key in (ord('g'), ord('i'), ord('I')):
                reset_session(set_mode=MODE_FWD)
                if key in (ord('i'), ord('I')):
                    recording = True; print("[CSV] Recording ON")
            elif key==ord('n'):
                sio.send_char(STEP_ONCE)              # 하드웨어 1회전
                pending_csv_rotate = True
                skip_log_this_frame = True
            elif key in (ord('o'), ord('O')):
                if key == ord('o'):
                    recording = False; print("[CSV] Recording PAUSED")
                enter_ret_manual()
            elif key==ord('x'):
                end_session("FORCE")
            elif key==ord('s'):
                if recording:
                    event_override = "S"; print("[CSV] Mark EVENT=S")
                else:
                    print("[CSV] Mark ignored (recording PAUSED)")

            elif key == ord('0'):
                sio.send_line('L0');
                print("[CMD] LED BLUE (0)")
            elif key == ord('1'):
                sio.send_line('L1');
                print("[CMD] LED RED (1)")


            elif key!=255:
                ch=chr(key)
                if ch.isprintable(): sio.send_char(ch)

            # 시리얼 수신
            pot_raw = sio.get_pot()
            ev_rx   = sio.pop_event()
            ev_tx   = sio.tx_last
            event_str = f"TX:{ev_tx}" if ev_tx else (ev_rx if ev_rx else "")

            # 호모그래피(최초 1회)
            if 'H_try' not in locals(): H_try=None
            if H is None and len(reds)==1 and len(blues)==3:
                H_try=build_H_from_points(reds, blues)
                if H_try is not None: H=H_try; prev_theta=None

            rpm=None; theta_deg=None; r_norm=None
            if H is not None and len(reds)==1:
                r_norm = cv2.perspectiveTransform(reds.reshape(1,1,2), H)[0,0]
                theta  = math.atan2(r_norm[1]-CENTER[1], r_norm[0]-CENTER[0])
                theta_deg = rad2deg(theta)
                if prev_theta is not None and dt>0:
                    dtheta = wrap_to_pi(theta - prev_theta)
                    omega  = dtheta/dt
                    omega_s = omega if omega_s is None else 0.8*omega_s + 0.2*omega
                    rpm = rad2deg(omega_s)/6.0
                    buf_rpm.append(rpm)
                    if len(buf_rpm) > STRIP_LEN: buf_rpm.pop(0)
                prev_theta = theta
            else:
                prev_theta=None

            # 세션 로직
            if session_on and theta_deg is not None:
                delta = abs((theta_deg - TRG_DEG + 180) % 360 - 180)
                in_band = (delta <= TOL_DEG)
                out_for_arm = (delta >= TOL_DEG + HYST_DEG)

                if in_band and armed:
                    if mode == MODE_FWD:
                        rot_count += 1
                        if rot_count >= ROTATIONS_TARGET:
                            enter_ret_auto(reason="TARGET")
                            event_str = f"AUTO_RET: rot={rot_count}"
                    elif mode == MODE_RET and ret_mode == RET_AUTO:
                        rot_count -= 1
                        if rot_count < 0: rot_count = 0
                    armed = False
                elif out_for_arm:
                    armed = True

                # 스톨(FWD)
                if mode == MODE_FWD:
                    abs_omega_deg_s = abs(rad2deg(omega_s)) if omega_s is not None else 0.0
                    if abs_omega_deg_s < STALL_OMEGA_THRESH:
                        stall_timer += dt
                    else:
                        stall_timer = 0.0

                    if stall_timer >= STALL_DWELL_SEC:
                        stall_timer = 0.0
                        stall_count += 1
                        if stall_count == 1 and not sent_T1:
                            thr1 = 375 + 40
                            sio.send_line(f'T{thr1}'); sent_T1=True; event_str=f"TX:T{thr1}"
                        elif stall_count == 2 and not sent_T2:
                            thr2 = 375 + 100
                            sio.send_line(f'T{thr2}'); sent_T2=True; event_str=f"TX:T{thr2}"
                        else:
                            enter_ret_auto(reason="STALL3")
                            event_str = "AUTO_RET: FAIL_STALL>2"

                # 복귀 완료(RET_AUTO)
                if mode == MODE_RET and ret_mode == RET_AUTO and rot_count <= 0 and r_norm is not None:
                    at_stop = False
                    if USE_STOP_ANGLE:
                        at_stop = is_at_stop_by_angle(theta_deg, STOP_THETA_DEG, STOP_TOL_DEG)
                    else:
                        at_stop = is_at_stop_by_xy(r_norm, STOP_PT, STOP_TOL_PX)
                    if at_stop and STOP_MAX_SPEED_DEG_S is not None and omega_s is not None:
                        at_stop = (abs(rad2deg(omega_s)) <= STOP_MAX_SPEED_DEG_S)
                    if at_stop:
                        sio.send_char(CMD_STOP)
                        rot_count = 0
                        session_on = False
                        ret_mode = RET_NONE
                        event_str="OK: return_to_stop"

                # 타임아웃
                if session_on and SESSION_TIMEOUT_S and (time.perf_counter()-session_t0) > SESSION_TIMEOUT_S:
                    end_session("TIMEOUT"); event_str="TIMEOUT"

            # 화면 표시
            mode_str = "FWD" if mode==MODE_FWD else f"RET-{'AUTO' if ret_mode==RET_AUTO else ('MANUAL' if ret_mode==RET_MANUAL else 'NONE')}"
            status = f"mode={mode_str} | rec={'ON' if recording else 'PAUSE'} | pot={'' if pot_raw is None else pot_raw} | rpm={'' if rpm is None else f'{rpm:.1f}'}"
            if session_on:
                status += f" | ROT={rot_count}/{ROTATIONS_TARGET} | STALL={stall_count}"
            cv2.putText(raw, status, (10,26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2, cv2.LINE_AA)
            if event_str:
                cv2.putText(raw, event_str[:64], (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180,255,180), 1, cv2.LINE_AA)
            cv2.imshow("raw", raw)

            # 플롯
            plot = init_plot()
            top1, bottom1 = MARGIN, PLOT_H - MARGIN
            rng = 1.3*max(30.0, max(map(abs, buf_rpm)) if buf_rpm else 0.0)
            draw_axes(plot, top1, bottom1, f"rpm(t) (±{rng:.0f})", -rng, rng)
            plot_series(plot, top1, bottom1, buf_rpm[-STRIP_LEN:], (0,255,0), -rng, rng)
            cv2.imshow("plots", plot)

            # CSV 기록
            if t0 is None: t0 = now
            t_sec = now - t0
            evt = event_override if event_override else event_str
            if recording and writer is not None and not skip_log_this_frame:
                writer.writerow([
                    f"{t_sec:.6f}",
                    "" if pot_raw is None else int(pot_raw),
                    "" if rpm is None else f"{rpm:.6f}",
                    "" if theta_deg is None else f"{theta_deg:.2f}",
                    rot_count if session_on else "",
                    stall_count if session_on else "",
                    evt
                ])

            # 프레임 종료시 회차 전환(+분석→LED)
            if pending_csv_rotate:
                if writer is not None:
                    writer.writerow([
                        f"{t_sec:.6f}",
                        "" if pot_raw is None else int(pot_raw),
                        "" if rpm is None else f"{rpm:.6f}",
                        "" if theta_deg is None else f"{theta_deg:.2f}",
                        rot_count if session_on else "",
                        stall_count if session_on else "",
                        "NEXT"
                    ])
                    csv_fp.flush()
                try: last_csv_path = Path(csv_fp.name)
                except Exception: last_csv_path = None
                try: csv_fp.close()
                except: pass

                # 분석 실행 (행수≥10) → LED 전송
                if last_csv_path is not None and last_csv_path.exists():
                    try:
                        rows = max(0, sum(1 for _ in open(last_csv_path, "r", encoding="utf-8")) - 1)
                    except Exception:
                        rows = 0
                    if rows >= 10:
                        print(f"[ANALYSIS] {last_csv_path.name} rows={rows}")
                        pred = run_inline_analysis(last_csv_path)
                        if pred in (0,1):
                            sio.send_line(f"L{pred}")   # ★ LED 지시
                            print(f"[LED] sent → L{pred} (0=RED, 1=BLUE)")
                    else:
                        print(f"[ANALYSIS] skip (rows<10): {last_csv_path.name}")

                # 새 CSV (대기) 오픈 → 다음 프레임부터 t=0
                _open_csv(start_paused=True)
                pending_csv_rotate = False
            skip_log_this_frame = False

    except Exception as e:
        print("[ERR] main loop exception:", repr(e))
    finally:
        cap.release(); cv2.destroyAllWindows()
        try:
            if writer: csv_fp.flush()
            if csv_fp: csv_fp.close()
        except: pass
        sio.close()
        print("[LOG] closed")

if __name__ == "__main__":
    main()
