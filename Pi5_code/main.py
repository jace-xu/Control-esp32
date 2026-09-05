#!/usr/bin/env python3
"""
主控程序 main.py
整合物料识别、靶子识别、串口通信协议

状态机：
  IDLE   → 等待 ESP32 命令
  TASK1  → 物料识别（持续发送物料坐标偏差）
  TASK2  → 靶子识别（持续发送靶子坐标偏差）

串口协议：
  接收：0xBB + cmd + color + 校验
  发送：0xAA + id1 + id2 + data1(int16 LE) + data2(int16 LE) + 校验

用法：
    正常模式: python3 main.py
    无串口调试: python3 main.py --no-serial
"""

import cv2
import numpy as np
import argparse
import sys
import time
import threading
from collections import deque, Counter

# ── 串口 ─────────────────────────────────────────────────────
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

RX_HEADER = 0xBB
TX_HEADER = 0xAA
CMD_STOP  = 0x00
CMD_TASK1 = 0x01
CMD_TASK2 = 0x02

COLOR_NONE  = 0x00
COLOR_GREEN = 0x01
COLOR_BLUE  = 0x02
COLOR_RED   = 0x03

COLOR_NAME_TO_ID = {"green": 1, "blue": 2, "red": 3, "unknown": 0}
COLOR_ID_TO_NAME = {1: "green", 2: "blue", 3: "red", 0: "unknown"}

# 图像中心
IMG_CX = 320
IMG_CY = 240

# ============================================================
# 串口工具函数
# ============================================================

def open_serial(port=SERIAL_PORT, baud=SERIAL_BAUD, timeout=0.1):
    return serial.Serial(port, baud, timeout=timeout)


def send_error(ser, id1, id2, data1, data2):
    data1 = max(-3000, min(3000, int(data1)))
    data2 = max(-3000, min(3000, int(data2)))
    payload = bytearray()
    payload.append(TX_HEADER)
    payload.append(id1 & 0xFF)
    payload.append(id2 & 0xFF)
    payload += int(data1).to_bytes(2, byteorder="little", signed=True)
    payload += int(data2).to_bytes(2, byteorder="little", signed=True)
    checksum = 0
    for b in payload:
        checksum ^= b
    payload.append(checksum)
    try:
        written = ser.write(bytes(payload))
        return written == len(payload)
    except Exception:
        return False


def receive_command(ser):
    while ser.in_waiting >= 4:
        head = ser.read(1)
        if not head or head[0] != RX_HEADER:
            continue
        rest = ser.read(3)
        if len(rest) < 3:
            return None
        cmd, color, checksum = rest[0], rest[1], rest[2]
        if ((RX_HEADER ^ cmd ^ color) == checksum and
                cmd in (CMD_STOP, CMD_TASK1, CMD_TASK2)):
            return cmd, color
    return None


# ============================================================
# 物料检测（不依赖颜色找圆，再用HSV分类颜色）
# ============================================================

OBJECT_COLOR_RANGES = {
    "red": [
        (np.array([0, 0, 180]), np.array([180, 60, 255])),
    ],
    "blue": [
        (np.array([108, 100, 35]),  np.array([126, 255, 210])),
    ],
    "green": [
        (np.array([48,  120, 45]),  np.array([72,  255, 255])),
    ],
}
OBJECT_COLOR_BGR = {"red": (0,0,220), "blue": (220,80,0),
                    "green": (0,160,0), "unknown": (160,160,160)}

# 移动平均滤波
_obj_cx_hist = deque(maxlen=8)
_obj_cy_hist = deque(maxlen=8)


def _preprocess(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l,a,b]), cv2.COLOR_LAB2BGR)


def _classify_object_color(frame, cx, cy, radius):
    """在检测到的圆形区域内采样HSV判断颜色"""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, W = hsv.shape[:2]
    r    = max(5, int(radius * 0.8))
    x1   = max(0, cx-r); y1 = max(0, cy-r)
    x2   = min(W, cx+r); y2 = min(H, cy+r)
    roi  = hsv[y1:y2, x1:x2]
    if roi.size == 0:
        return "unknown"
    h_roi, w_roi = roi.shape[:2]
    mask = np.zeros((h_roi, w_roi), dtype=np.uint8)
    cv2.circle(mask, (w_roi//2, h_roi//2), r, 255, -1)
    total = int(np.count_nonzero(mask))
    if total == 0:
        return "unknown"
    scores = {}
    for color, ranges in OBJECT_COLOR_RANGES.items():
        cm = np.zeros((h_roi, w_roi), dtype=np.uint8)
        for lo, hi in ranges:
            cm |= cv2.inRange(roi, lo, hi)
        scores[color] = int(np.count_nonzero(cv2.bitwise_and(cm, mask)))
    best = max(scores, key=scores.get)
    return best if scores[best]/total >= 0.08 else "unknown"


def find_object(frame):
    """
    检测物料顶面圆心。
    主方案：霍夫圆。备用：距离变换。
    返回 (cx, cy, radius, color) 或 None。
    """
    H, W     = frame.shape[:2]
    frame_eq = _preprocess(frame)
    gray     = cv2.cvtColor(frame_eq, cv2.COLOR_BGR2GRAY)
    blurred  = cv2.GaussianBlur(gray, (9,9), 2)

    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT_ALT,
        dp=1.5, minDist=50,
        param1=80, param2=0.70,
        minRadius=30, maxRadius=400)

    cx, cy, r = None, None, None

    if circles is not None:
        result = []
        for ccx, ccy, cr in np.round(circles[0]).astype(int):
            result.append((int(ccx), int(ccy), float(cr)))
        if result:
            max_r   = max(c[2] for c in result)
            cands   = [c for c in result if c[2] >= max_r*0.8]
            best    = max(cands, key=lambda c: np.hypot(
                c[0]-W//2, c[1]-H//2))
            cx, cy, r = best

    if cx is None:
        # 备用：距离变换
        edges = cv2.Canny(blurred, 30, 100)
        inv   = cv2.bitwise_not(edges)
        dist  = cv2.distanceTransform(inv, cv2.DIST_L2, 5)
        _, max_val, _, max_loc = cv2.minMaxLoc(dist)
        cx, cy = max_loc
        r = max(int(max_val * 0.9), min(W,H)//3)

    # 移动平均平滑
    _obj_cx_hist.append(cx)
    _obj_cy_hist.append(cy)
    cx = int(sum(_obj_cx_hist)/len(_obj_cx_hist))
    cy = int(sum(_obj_cy_hist)/len(_obj_cy_hist))

    # 颜色识别
    color = _classify_object_color(frame, cx, cy, r)

    return cx, cy, r, color


# ============================================================
# 靶子检测（v12 SmoothTracker）
# ============================================================

TARGET_COLOR_RANGES = {
    "red": [
        (np.array([0,   40,  0]),  np.array([12,  255, 255])),
        (np.array([154, 40,  0]),  np.array([180, 255, 255])),
    ],
    "blue": [
        (np.array([90,  20,  0]),  np.array([149, 255, 255])),
    ],
    "green": [
        (np.array([43,  25,  0]),  np.array([92,  255, 255])),
    ],
}
TARGET_COLOR_BGR = {
    "red": (0,0,220), "blue": (220,80,0),
    "green": (0,160,0), "unknown": (160,160,160),
}

class SmoothTracker:
    def __init__(self, cx, cy, smooth=15):
        self.history     = [(cx,cy)] * smooth
        self.smooth      = smooth
        self.lost_frames = 0
        self.color       = "unknown"

    def update(self, cx, cy):
        self.history.append((cx,cy))
        if len(self.history) > self.smooth:
            self.history.pop(0)
        self.lost_frames = 0
        return (int(sum(p[0] for p in self.history)/len(self.history)),
                int(sum(p[1] for p in self.history)/len(self.history)))

    def predict(self):
        return (sum(p[0] for p in self.history)/len(self.history),
                sum(p[1] for p in self.history)/len(self.history))


class TargetDetector:
    def __init__(self, min_radius=15, max_radius=300,
                 min_rings=2, cluster_tol=0.45, max_lost=10):
        self.min_radius  = min_radius
        self.max_radius  = max_radius
        self.min_rings   = min_rings
        self.cluster_tol = cluster_tol
        self.max_lost    = max_lost
        self._clahe      = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        self._trackers   = {}
        self._next_id    = 0

    def _preprocess(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self._clahe.apply(l)
        return cv2.cvtColor(cv2.merge([l,a,b]), cv2.COLOR_LAB2BGR)

    def _enhance_edges(self, gray):
        eroded  = cv2.erode(gray, None, iterations=2)
        dilated = cv2.dilate(eroded, np.ones((7,7),np.uint8), iterations=1)
        clahe   = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8,8))
        clahed  = clahe.apply(dilated)
        k5      = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        grad    = cv2.morphologyEx(clahed, cv2.MORPH_GRADIENT, k5)
        b1      = cv2.GaussianBlur(grad,(7,7),3)
        enh     = cv2.convertScaleAbs(b1, alpha=4, beta=0)
        b2      = cv2.GaussianBlur(enh,(7,7),3)
        return cv2.adaptiveThreshold(
            b2,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,11,2)

    def _detect_circles(self, enhanced):
        circles = cv2.HoughCircles(
            enhanced, cv2.HOUGH_GRADIENT_ALT,
            dp=1.5, minDist=10,
            param1=80, param2=0.75,
            minRadius=self.min_radius,
            maxRadius=self.max_radius)
        if circles is None:
            return np.empty((0,3), dtype=np.float32)
        return np.round(circles[0]).astype(np.float32)

    def _cluster(self, circles):
        if len(circles)==0: return []
        used  = [False]*len(circles)
        order = np.argsort(-circles[:,2])
        groups= []
        for i in order:
            if used[i]: continue
            cx,cy,cr = circles[i]
            grp = [(cx,cy,cr)]; used[i]=True
            for j in order:
                if used[j]: continue
                cx2,cy2,cr2 = circles[j]
                if np.hypot(cx-cx2,cy-cy2)<max(cr,cr2)*self.cluster_tol:
                    grp.append((cx2,cy2,cr2)); used[j]=True
            if len(grp)<self.min_rings: continue
            ws  = np.array([g[2] for g in grp],dtype=np.float32); ws/=ws.sum()
            mcx = float(np.sum([g[0]*w for g,w in zip(grp,ws)]))
            mcy = float(np.sum([g[1]*w for g,w in zip(grp,ws)]))
            groups.append({"center":(mcx,mcy),
                           "radii":sorted([float(g[2]) for g in grp])})
        return groups

    def _classify_color(self, hsv, cx, cy, r_min, r_max):
        H,W = hsv.shape[:2]
        pad = int(r_max)+2
        x1=max(0,int(cx)-pad); y1=max(0,int(cy)-pad)
        x2=min(W,int(cx)+pad); y2=min(H,int(cy)+pad)
        roi=hsv[y1:y2,x1:x2]
        if roi.size==0: return "unknown"
        h,w=roi.shape[:2]; lx,ly=int(cx)-x1,int(cy)-y1
        mo=np.zeros((h,w),np.uint8); mi=np.zeros((h,w),np.uint8)
        cv2.circle(mo,(lx,ly),int(r_max),255,-1)
        cv2.circle(mi,(lx,ly),max(1,int(r_min*0.8)),255,-1)
        ring=cv2.subtract(mo,mi); total=int(np.count_nonzero(ring))
        if total==0: return "unknown"
        scores={}
        for color,ranges in TARGET_COLOR_RANGES.items():
            cm=np.zeros((h,w),np.uint8)
            for lo,hi in ranges: cm|=cv2.inRange(roi,lo,hi)
            scores[color]=int(np.count_nonzero(cv2.bitwise_and(cm,ring)))
        best=max(scores,key=scores.get)
        return best if scores[best]/total>=0.02 else "unknown"

    def _update_trackers(self, detections):
        MATCH_DIST=80
        preds={tid:tk.predict() for tid,tk in self._trackers.items()}
        for tk in self._trackers.values(): tk.lost_frames+=1
        matched=set(); results=[]
        for det in detections:
            raw_cx,raw_cy=float(det["cx"]),float(det["cy"])
            color,radii=det["color"],det["radii"]
            best_tid=None; best_dist=MATCH_DIST
            for tid,(px,py) in preds.items():
                d=np.hypot(raw_cx-px,raw_cy-py)
                if d<best_dist and tid not in matched:
                    best_dist=d; best_tid=tid
            if best_tid is not None:
                tk=self._trackers[best_tid]
                sx,sy=tk.update(raw_cx,raw_cy)
                if color!="unknown": tk.color=color
                matched.add(best_tid)
            else:
                tk=SmoothTracker(raw_cx,raw_cy); tk.color=color
                tid=self._next_id; self._next_id+=1
                self._trackers[tid]=tk
                sx,sy=tk.update(raw_cx,raw_cy); matched.add(tid)
            results.append({"color":tk.color,"cx":sx,"cy":sy,"radii":radii})
        dead=[t for t,tk in self._trackers.items()
              if tk.lost_frames>self.max_lost]
        for t in dead: del self._trackers[t]
        return results

    def detect(self, frame):
        frame_eq=self._preprocess(frame)
        scale=0.5
        small=cv2.resize(frame_eq,(0,0),fx=scale,fy=scale)
        gray =cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
        hsv  =cv2.cvtColor(small,cv2.COLOR_BGR2HSV)
        enhanced=self._enhance_edges(gray)
        circles =self._detect_circles(enhanced)
        groups  =self._cluster(circles)
        detections=[]
        for g in groups:
            cx_s,cy_s=g["center"]; radii_s=g["radii"]
            color=self._classify_color(hsv,cx_s,cy_s,radii_s[0],radii_s[-1])
            detections.append({
                "cx":cx_s/scale,"cy":cy_s/scale,
                "color":color,
                "radii":sorted([r/scale for r in radii_s])[:6],
            })
        return self._update_trackers(detections)


# ============================================================
# 主状态机
# ============================================================

STATE_IDLE  = "IDLE"
STATE_TASK1 = "TASK1"
STATE_TASK2 = "TASK2"

def run(no_serial=False, show_debug=True, auto_task1=False, auto_task2=False,
    force_task1=False, force_task2=False):
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] 无法打开摄像头"); sys.exit(1)
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"摄像头: {W}x{H}")

    # 打开串口
    ser = None
    if not no_serial:
        if not SERIAL_AVAILABLE:
            print("[ERROR] pyserial未安装，请用 --no-serial 模式")
            sys.exit(1)
        try:
            ser = open_serial()
            print(f"串口: {SERIAL_PORT} @ {SERIAL_BAUD}")
        except Exception as e:
            print(f"[ERROR] 串口打开失败: {e}")
            sys.exit(1)
    else:
        print("[调试] 无串口模式，键盘模拟命令：1=物料 2=靶子(红) q=停止")

    target_detector = TargetDetector()

    if auto_task1 and auto_task2:
        auto_task2 = False
    if force_task1:
        auto_task1 = True
        auto_task2 = False
    if force_task2:
        auto_task2 = True
        auto_task1 = False
    state        = STATE_TASK1 if auto_task1 else (STATE_TASK2 if auto_task2 else STATE_IDLE)
    task2_color  = COLOR_RED if auto_task2 else 0  # ESP32指定的靶子颜色id
    # 靶子颜色确认缓冲：收集3秒投票
    color_buf     = []
    confirm_start = None
    confirmed     = False      # 是否已确认并发送过9
    confirm_done  = False      # 是否已完成首次3s确认流程

    print(f"状态: {state}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 读帧失败")
            continue

        vis = frame.copy()

        # ── 读串口命令 ────────────────────────────────────
        if ser:
            result = receive_command(ser)
            if result is not None:
                cmd, color = result
                if cmd == CMD_STOP:
                    state = STATE_IDLE
                    color_buf.clear(); confirm_start=None; confirmed=False; confirm_done=False
                    print(f"[串口] STOP → IDLE")
                elif cmd == CMD_TASK1:
                    state = STATE_TASK1
                    _obj_cx_hist.clear(); _obj_cy_hist.clear()
                    print(f"[串口] TASK1 → 物料识别")
                elif cmd == CMD_TASK2:
                    state       = STATE_TASK2
                    task2_color = color
                    color_buf.clear(); confirm_start=time.time()
                    confirmed=False; confirm_done=False
                    print(f"[串口] TASK2 color={COLOR_ID_TO_NAME.get(color,'?')} → 靶子识别")
        else:
            # 键盘模拟
            key = cv2.waitKey(1) & 0xFF
            if key == ord('1'):
                state = STATE_TASK1
                _obj_cx_hist.clear(); _obj_cy_hist.clear()
                print("模拟: TASK1")
            elif key == ord('2'):
                state = STATE_TASK2; task2_color = COLOR_RED
                color_buf.clear(); confirm_start=time.time()
                confirmed=False; confirm_done=False
                print("模拟: TASK2 (红色)")
            elif key == ord('q'):
                state = STATE_IDLE
                print("模拟: STOP")
            elif key == 27:   # ESC退出
                break

        # ── TASK1：物料识别 ───────────────────────────────
        if state == STATE_TASK1:
            cx, cy, r, color = find_object(frame)
            dx = cx - IMG_CX
            dy = cy - IMG_CY
            color_id = COLOR_NAME_TO_ID.get(color, 0)

            # 发送
            if ser:
                send_error(ser, 1, color_id, dx, dy)

            # 可视化
            bgr = OBJECT_COLOR_BGR.get(color, (160,160,160))
            cv2.circle(vis, (cx,cy), int(r), bgr, 2)
            cv2.circle(vis, (cx,cy), 6, bgr, -1)
            cv2.drawMarker(vis,(IMG_CX,IMG_CY),(255,255,255),
                           cv2.MARKER_CROSS,30,2)
            cv2.arrowedLine(vis,(IMG_CX,IMG_CY),(cx,cy),(255,255,0),2)
            cv2.putText(vis,
                        f"TASK1 {color} ({cx},{cy}) dx={dx:+d} dy={dy:+d}",
                        (8,30),cv2.FONT_HERSHEY_SIMPLEX,0.6,bgr,2)
            print(f"[TASK1] {color}(id={color_id}) dx={dx:+d} dy={dy:+d}")

        # ── TASK2：靶子识别 ───────────────────────────────
        elif state == STATE_TASK2:
            if confirm_start is None:
                confirm_start = time.time()
            targets = target_detector.detect(frame)
            target_color_name = COLOR_ID_TO_NAME.get(task2_color, "unknown")

            # 找和ESP32指定颜色一致的靶子
            matched = [t for t in targets
                       if t["color"] == target_color_name]

            cv2.drawMarker(vis,(IMG_CX,IMG_CY),(255,255,255),
                           cv2.MARKER_CROSS,30,2)

            if not confirm_done and targets:
                for t in targets:
                    if t["color"] != "unknown":
                        color_buf.append(t["color"])

            elapsed = time.time() - confirm_start
            if not confirm_done and elapsed >= 3.0:
                if color_buf:
                    vote_color = Counter(color_buf).most_common(1)[0][0]
                    if vote_color == target_color_name:
                        confirmed = True
                        print(f"[TASK2] 确认! {vote_color}")
                    else:
                        print(f"[TASK2] 颜色不一致 vote={vote_color} expected={target_color_name}")
                confirm_done = True

            if matched:
                t   = matched[0]
                cx  = t["cx"]; cy = t["cy"]
                dx  = cx - IMG_CX
                dy  = cy - IMG_CY
                bgr = TARGET_COLOR_BGR.get(t["color"],(160,160,160))
            else:
                cx = cy = dx = dy = 0
                bgr = (160,160,160)

            if confirmed and matched:
                if ser:
                    send_error(ser, 2, 9, dx, dy)
            else:
                if ser:
                    send_error(ser, 2, 0, 0, 0)

            if matched:
                for rv in t["radii"][:6]:
                    cv2.circle(vis,(cx,cy),int(rv),bgr,2)
                cv2.circle(vis,(cx,cy),6,bgr,-1)
                cv2.arrowedLine(vis,(IMG_CX,IMG_CY),(cx,cy),(255,255,0),2)

            if confirmed:
                status = "✓确认"
            elif confirm_done:
                status = "未确认"
            else:
                status = f"投票{elapsed:.1f}s"

            if not matched:
                cv2.putText(vis,
                            f"TASK2 找不到 {target_color_name}",
                            (8,30),cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,(160,160,160),2)
            else:
                cv2.putText(vis,
                            f"TASK2 {t['color']} ({cx},{cy}) "
                            f"dx={dx:+d} dy={dy:+d} [{status}]",
                            (8,30),cv2.FONT_HERSHEY_SIMPLEX,0.6,bgr,2)

        # ── IDLE ─────────────────────────────────────────
        else:
            cv2.putText(vis,"IDLE - 等待命令",
                        (8,30),cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,(200,200,200),2)

        # 状态显示
        cv2.putText(vis, f"state:{state}",
                    (8,H-10),cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,(200,200,200),1)

        if show_debug:
            cv2.imshow("Main (ESC=quit)", vis)
            if not no_serial:
                if cv2.waitKey(1) & 0xFF == 27:
                    break

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-serial",  action="store_true",
                        help="无串口调试模式，键盘模拟命令")
    parser.add_argument("--auto-task1", action="store_true",
                        help="启动后直接进入TASK1（便于无串口测试）")
    parser.add_argument("--auto-task2", action="store_true",
                        help="启动后直接进入TASK2（默认红色靶子，用于无串口测试）")
    parser.add_argument("--force-task1", action="store_true",
                        help="不等ESP32命令，直接进入TASK1并走串口发送")
    parser.add_argument("--force-task2", action="store_true",
                        help="不等ESP32命令，直接进入TASK2并走串口发送")
    parser.add_argument("--no-display", action="store_true",
                        help="不显示画面（无头模式）")
    args = parser.parse_args()
    run(no_serial=args.no_serial,
        show_debug=not args.no_display,
        auto_task1=args.auto_task1,
        auto_task2=args.auto_task2,
        force_task1=args.force_task1,
        force_task2=args.force_task2)


if __name__ == "__main__":
    main()