"""ROPI 키보드 조종.

방향키(또는 WASD)로 주행하고, 어깨 모터`로 공격한다.

두 가지 모드가 있다.
  HOLD  - 키를 누르고 있는 동안만 움직인다. 떼면 멈춘다. (기본값)
  LATCH - 한 번 누르면 계속 간다. 스페이스로 멈춘다.

HOLD 가 기본인 이유: 로봇이 손을 떠나 굴러가는 사고를 막기 위해서다.
터미널의 키 자동반복을 이용하므로 키를 누르고 있으면 계속 굴러간다.
"""
import os
import select
import sys
import termios
import threading
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ropi_motion as m

IDLE_STOP = 0.8      # HOLD 모드에서 이 시간 동안 키가 없으면 정지(초)

ARROWS = {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}

HELP = """
=============== ROPI 키보드 조종 ===============
[주행]
  위 / W        전진
  아래 / S      후진
  왼쪽 / A      좌회전 (곡선)
  오른쪽 / D    우회전 (곡선)
  Q             제자리 좌회전
  E             제자리 우회전
  SPACE         정지

[속도]
  + 또는 ]      속도 올리기
  - 또는 [      속도 내리기

[공격]
  F             양팔 공격
  J             왼팔        K  오른팔
  G             콤보 (왼-오른-양팔)

[인사]
  H             양손 흔들기
  U             왼손        I  오른손
  Y             손 흔드는 방향 반전

[설정]
  M             HOLD <-> LATCH 모드 전환
  R             양쪽 앞뒤 반전 (전진 눌렀는데 둘 다 뒤로 가면)
  1 / 2         왼바퀴만 / 오른바퀴만 반전 (한쪽만 거꾸로 돌 때)
  T             팔 방향 반전
  , / .         왼바퀴 보정 -/+     (직진이 쏠릴 때)
  ; / '         오른바퀴 보정 -/+
  N / B         회전 반경 크게 / 작게
                 (1.0=거의 직진  0.0=안쪽바퀴 정지  -1.0=제자리회전)
  P             지금 설정 저장
  ?             도움말 다시 보기
  X             종료
================================================
"""

lock = threading.Lock()
last_key_time = time.monotonic()
driving = False
mode_hold = True
running = True


def status():
    mode = "HOLD (누르고 있는 동안)" if mode_hold else "LATCH (누르면 계속)"
    print(f"  속도={m.SPEED}  회전비={m.TURN_RATIO:.2f}  "
          f"보정 L={m.LEFT_TRIM:.2f} R={m.RIGHT_TRIM:.2f}  "
          f"방향 L={m.LEFT_FLIP:+d} R={m.RIGHT_FLIP:+d} 전체={m.WHEEL_FLIP:+d}  "
          f"팔={m.ATTACK_FLIP:+d}  |  {mode}")


def go(action, label):
    """주행 명령. HOLD 모드면 워치독이 알아서 끊는다."""
    global driving, last_key_time
    with lock:
        action()
        driving = True
        last_key_time = time.monotonic()
    print(f"  {label}  (속도 {m.SPEED})")


def halt(label="정지"):
    global driving
    with lock:
        m.stop_wheels()
        driving = False
    print(f"  {label}")


def do_attack(action, label):
    """공격 중에는 바퀴를 멈춰 세운다. 팔만 움직여야 자세가 안 흐트러진다."""
    global driving, last_key_time
    with lock:
        m.stop_wheels()
        driving = False
    print(f"  {label}")
    action()
    with lock:
        last_key_time = time.monotonic()


def watchdog():
    """HOLD 모드에서 키가 끊기면 바퀴를 멈춘다.

    이게 없으면 키를 뗀 뒤에도 PCA9685 가 PWM 을 유지해 로봇이 계속 굴러간다.
    """
    global driving
    while running:
        time.sleep(0.05)
        if not mode_hold:
            continue
        with lock:
            if driving and (time.monotonic() - last_key_time) > IDLE_STOP:
                m.stop_wheels()
                driving = False


def read_key():
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    # 방향키는 ESC [ A 같은 3바이트 시퀀스로 들어온다.
    if select.select([sys.stdin], [], [], 0.05)[0]:
        return ARROWS.get(sys.stdin.read(2))
    return "ESC"


def main():
    global mode_hold, running, last_key_time

    if not sys.stdin.isatty():
        print("터미널에서 직접 실행하세요.")
        return

    m.load_settings()
    print(HELP)
    status()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    termios.tcflush(fd, termios.TCIFLUSH)
    tty.setcbreak(fd)

    guard = threading.Thread(target=watchdog, daemon=True)
    guard.start()

    try:
        while True:
            key = read_key()
            if key is None:
                continue
            k = key.lower() if len(key) == 1 else key

            # --- 주행 ---
            if k in ("UP", "w"):
                go(m.forward, "전진")
            elif k in ("DOWN", "s"):
                go(m.backward, "후진")
            elif k in ("LEFT", "a"):
                go(m.arc_left, "좌회전")
            elif k in ("RIGHT", "d"):
                go(m.arc_right, "우회전")
            elif k == "q":
                go(m.spin_left, "제자리 좌회전")
            elif k == "e":
                go(m.spin_right, "제자리 우회전")
            elif k == " ":
                halt()

            # --- 속도 ---
            elif k in ("+", "=", "]"):
                m.speed_up(); status()
            elif k in ("-", "_", "["):
                m.speed_down(); status()

            # --- 공격 ---
            elif k == "f":
                do_attack(m.attack, "양팔 공격")
            elif k == "j":
                do_attack(m.attack_left, "왼팔 공격")
            elif k == "k":
                do_attack(m.attack_right, "오른팔 공격")
            elif k == "g":
                do_attack(m.attack_combo, "콤보")

            # --- 인사 ---
            elif k == "h":
                do_attack(m.wave_both, "양손 흔들기")
            elif k == "u":
                do_attack(m.wave_left, "왼손 흔들기")
            elif k == "i":
                do_attack(m.wave_right, "오른손 흔들기")
            elif k == "y":
                m.WAVE_FLIP = -m.WAVE_FLIP
                print("  손 흔드는 방향 반전"); status()

            # --- 설정 ---
            elif k == "m":
                mode_hold = not mode_hold
                halt("모드 전환")
                status()
            elif k == "r":
                m.WHEEL_FLIP = -m.WHEEL_FLIP
                halt("양쪽 앞뒤 반전"); status()
            elif k == "1":
                m.LEFT_FLIP = -m.LEFT_FLIP
                halt("왼바퀴 반전"); status()
            elif k == "2":
                m.RIGHT_FLIP = -m.RIGHT_FLIP
                halt("오른바퀴 반전"); status()
            elif k == "t":
                m.ATTACK_FLIP = -m.ATTACK_FLIP
                print("  팔 방향 반전"); status()
            elif k == ",":
                m.LEFT_TRIM = round(max(0.2, m.LEFT_TRIM - 0.05), 2); status()
            elif k == ".":
                m.LEFT_TRIM = round(min(1.5, m.LEFT_TRIM + 0.05), 2); status()
            elif k == ";":
                m.RIGHT_TRIM = round(max(0.2, m.RIGHT_TRIM - 0.05), 2); status()
            elif k == "'":
                m.RIGHT_TRIM = round(min(1.5, m.RIGHT_TRIM + 0.05), 2); status()
            elif k == "n":
                m.TURN_RATIO = round(min(1.0, m.TURN_RATIO + 0.05), 2); status()
            elif k == "b":
                m.TURN_RATIO = round(max(-1.0, m.TURN_RATIO - 0.05), 2); status()
            elif k == "p":
                print("  저장:", m.save_settings()); status()
            elif k == "?":
                print(HELP); status()
            elif k == "z":
                do_attack(m.spin_dance, "Spin Dance")

            elif k == "c":
                do_attack(m.robot_dance, "Robot Dance")

            elif k == "x":
                break
            elif k in ("\r", "\n"):
                status()
            else:
                print(f"  ? '{key}'")
    finally:
        running = False
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        m.stop()
        print("\n전부 정지하고 종료합니다.")


if __name__ == "__main__":
    main()
