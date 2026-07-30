"""ROPI 주행 + 공격 모션.

발목 모터에 바퀴를 달아 차동 구동(differential drive)으로 바뀌었다.
골반 모터는 고정되어 있으므로 이 파일은 절대 건드리지 않는다.
어깨 모터는 공격 동작에 쓴다.

CR 서보에서 offset 은 '각도'가 아니라 '회전 속도'다.
바퀴에서는 그게 오히려 자연스럽다. offset 이 곧 주행 속도다.

PCA9685 는 프로세스가 끝나도 마지막 PWM 을 유지한다.
그래서 주행 명령은 값만 걸어놓고 바로 끝내면 되고,
멈출 때만 PWM 을 끊어주면 된다.
"""
import json
import os
import sys
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

# --- 채널 ---------------------------------------------------------
LEFT_HIP = 0        # 고정됨. 절대 구동하지 않는다.
RIGHT_HIP = 1       # 고정됨. 절대 구동하지 않는다.
LEFT_WHEEL = 2      # 예전 왼발목
RIGHT_WHEEL = 3     # 예전 오른발목
LEFT_SHOULDER = 4
RIGHT_SHOULDER = 5
LEFT_WRIST = 6      # 왼손
RIGHT_WRIST = 7     # 오른손

# 채널이 어느 관절인지는 로봇마다 다를 수 있다.
# (예: 팔이 4~7 이 아니라 8~11 에 꽂힌 개체가 있었다)
# 그래서 아래 값들은 ropi_drive.json 으로 덮어쓸 수 있게 해뒀다.
#   python3 ropi_motion.py channels 8 9 10 11
CHANNEL_KEYS = ("LEFT_WHEEL", "RIGHT_WHEEL",
                "LEFT_SHOULDER", "RIGHT_SHOULDER",
                "LEFT_WRIST", "RIGHT_WRIST")

# 관절별 회전 부호. 좌우가 마주보게 달린 곳은 부호를 뒤집어야 같이 움직인다.
DIR_SIGNS = {
    "LEFT_WHEEL": 1, "RIGHT_WHEEL": -1,
    "LEFT_SHOULDER": 1, "RIGHT_SHOULDER": 1,
    "LEFT_WRIST": 1, "RIGHT_WRIST": 1,
}

WHEELS = SHOULDERS = WRISTS = ARMS = ALL_DRIVEN = []
CENTER = {}
DIRECTION = {}


def rebuild():
    """채널 번호가 바뀌었을 때 파생 목록을 다시 만든다."""
    global WHEELS, SHOULDERS, WRISTS, ARMS, ALL_DRIVEN, CENTER, DIRECTION
    g = globals()

    WHEELS = [g["LEFT_WHEEL"], g["RIGHT_WHEEL"]]
    SHOULDERS = [g["LEFT_SHOULDER"], g["RIGHT_SHOULDER"]]
    WRISTS = [g["LEFT_WRIST"], g["RIGHT_WRIST"]]
    ARMS = SHOULDERS + WRISTS
    ALL_DRIVEN = WHEELS + ARMS          # 골반은 고정이라 여기에 넣지 않는다

    CENTER = {ch: 90 for ch in range(16)}
    DIRECTION = {ch: 1 for ch in range(16)}
    for key, sign in DIR_SIGNS.items():
        DIRECTION[g[key]] = sign


rebuild()

# --- 주행 설정 ----------------------------------------------------
SPEED = 100         # 주행 속도 0~100 (%). 100 이 펄스 범위의 끝이다.

# 속도는 퍼센트로 다루고, 실제로는 아래 offset 으로 변환해서 서보에 보낸다.
# offset 90 이 중립(1500us)에서 끝(2250us)까지의 최대치다.
FULL_OFFSET = 90.0

# 좌/우회전 시 안쪽 바퀴를 어떻게 할 것인가.
#   1.0  = 바깥과 같은 속도 (사실상 직진)
#   0.5  = 절반 속도로 완만하게
#   0.0  = 완전히 세운다. 그 바퀴를 축으로 돈다.  <- 지금 설정
#  -1.0  = 반대로 돌린다 (= 제자리 회전)
TURN_RATIO = 0.0
WHEEL_FLIP = 1      # 양쪽 다 앞뒤가 반대면 -1
LEFT_FLIP = -1      # 왼바퀴만 반대로 돌 때 -1 (바퀴를 거꾸로 단 경우)
RIGHT_FLIP = 1      # 오른바퀴만 반대로 돌 때 -1
LEFT_TRIM = 1.00    # 직진이 한쪽으로 쏠릴 때 좌우 바퀴 보정
RIGHT_TRIM = 1.00

# --- 공격 설정 ----------------------------------------------------
ATTACK_SPEED = 100  # 팔 휘두르는 속도 0~100 (%)
ATTACK_TIME = 0.22  # 한 방향으로 휘두르는 시간 -> 팔이 도는 각도
ATTACK_FLIP = 1     # 팔이 반대로 나가면 -1

# --- 손 흔들기 (인사) ---------------------------------------------
WAVE_SPEED = 100    # 손 흔드는 속도 0~100 (%)
WAVE_LIFT = 0.30    # 손을 올리는 시간 -> 얼마나 높이 드는가
WAVE_TIME = 0.16    # 한 번 흔드는 시간 -> 흔드는 폭
WAVE_COUNT = 3      # 왕복 횟수
WAVE_FLIP = 1       # 흔드는 방향이 어색하면 -1

MIN_SPEED = 20
MAX_SPEED = 100
SPEED_STEP = 10

SETTINGS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "ropi_drive.json")
SAVED_KEYS = ("SPEED", "TURN_RATIO", "WHEEL_FLIP", "LEFT_FLIP", "RIGHT_FLIP",
              "LEFT_TRIM", "RIGHT_TRIM",
              "ATTACK_SPEED", "ATTACK_TIME", "ATTACK_FLIP",
              "WAVE_SPEED", "WAVE_LIFT", "WAVE_TIME", "WAVE_COUNT",
              "WAVE_FLIP") + CHANNEL_KEYS


def load_settings():
    """저장된 설정을 불러온다. 없으면 위의 기본값을 그대로 쓴다.

    주행 명령은 매번 새 프로세스로 실행되므로(서버가 그렇게 부른다)
    속도 같은 값은 파일에 남겨둬야 다음 실행에서도 유지된다.
    """
    try:
        with open(SETTINGS_PATH, encoding="utf-8") as fp:
            data = json.load(fp)
    except (OSError, ValueError):
        return
    g = globals()
    for key in SAVED_KEYS:
        if key in data:
            g[key] = data[key]
    rebuild()      # 채널이 바뀌었을 수 있으므로 파생 목록을 다시 만든다


def save_settings():
    g = globals()
    data = {key: g[key] for key in SAVED_KEYS}
    with open(SETTINGS_PATH, "w", encoding="utf-8") as fp:
        json.dump(data, fp, indent=2)
    return SETTINGS_PATH


def clamp(angle):
    return max(0, min(180, angle))


def set_servo(channel, angle):
    kit.servo[channel].angle = clamp(angle)


def move_servo(channel, offset):
    set_servo(channel, CENTER[channel] + offset * DIRECTION[channel])


def stop_channels(channels):
    """PWM 을 끊어 확실히 멈춘다.

    CR 서보에서 1500us 는 정지 신호가 아니라 그냥 하나의 회전 명령이다.
    정지점이 1500us 가 아닌 서보는 계속 돌아버린다. 펄스를 끊어야 멈춘다.
    """
    for ch in channels:
        kit._pca.channels[ch].duty_cycle = 0


# --- 주행 ---------------------------------------------------------

def pct_to_offset(pct):
    """속도 퍼센트(0~100)를 서보 offset 으로 바꾼다.

    100% 가 offset 90, 즉 펄스 범위의 끝(2250us)이다.
    CR 서보는 대개 2000us 부근에서 이미 최고 속도에 도달하므로
    90% 와 100% 는 체감상 같을 수 있다.
    """
    return FULL_OFFSET * pct / 100.0


def _wheel(channel, ratio, trim, flip):
    """바퀴 하나에 속도를 건다. ratio 가 0 이면 PWM 을 끊는다.

    offset 0 을 주면 1500us 가 나가는데, CR 서보에서 1500us 는 정지가 아니라
    그냥 하나의 회전 명령이다. 정지점이 1500us 가 아닌 서보는 계속 돌아버린다.
    확실히 세우려면 펄스를 끊는 수밖에 없다.
    """
    if abs(ratio) < 0.01:
        stop_channels([channel])
    else:
        move_servo(channel,
                   pct_to_offset(SPEED) * ratio * trim * flip * WHEEL_FLIP)


def drive(left, right):
    """left/right: -1.0 ~ +1.0. 양수가 전진, 0 이면 그 바퀴는 완전히 선다.

    값을 걸어놓고 바로 반환한다. PCA9685 가 PWM 을 유지하므로
    stop() 을 부르기 전까지 바퀴는 계속 돈다.
    """
    _wheel(LEFT_WHEEL, left, LEFT_TRIM, LEFT_FLIP)
    _wheel(RIGHT_WHEEL, right, RIGHT_TRIM, RIGHT_FLIP)


def forward():
    drive(1.0, 1.0)


def backward():
    drive(-1.0, -1.0)


def arc_left():
    """좌회전. 안쪽(왼) 바퀴를 세우고 그 바퀴를 축으로 돈다."""
    drive(TURN_RATIO, 1.0)


def arc_right():
    """우회전. 안쪽(오른) 바퀴를 세운다."""
    drive(1.0, TURN_RATIO)


def spin_left():
    """제자리 좌회전. 두 바퀴가 반대로 돈다."""
    drive(-1.0, 1.0)


def spin_right():
    drive(1.0, -1.0)


def stop_wheels():
    stop_channels(WHEELS)


def stop():
    """바퀴와 팔 전부 정지. 골반은 고정이라 건드리지 않는다."""
    stop_channels(ALL_DRIVEN)


def home():
    stop()


def set_speed(value):
    global SPEED
    SPEED = int(max(MIN_SPEED, min(MAX_SPEED, value)))
    return SPEED


def speed_up():
    return set_speed(SPEED + SPEED_STEP)


def speed_down():
    return set_speed(SPEED - SPEED_STEP)


# --- 공격 ---------------------------------------------------------

def swing(channels, seconds=None):
    """팔을 앞으로 휘둘렀다가 되돌린다.

    CR 서보라 위치 개념이 없다. 나간 시간만큼 되돌려야 제자리로 온다.
    """
    sec = ATTACK_TIME if seconds is None else seconds
    power = pct_to_offset(ATTACK_SPEED) * ATTACK_FLIP

    for ch in channels:
        move_servo(ch, power)
    time.sleep(sec)

    for ch in channels:
        move_servo(ch, -power)
    time.sleep(sec)

    stop_channels(channels)


def attack_left():
    swing([LEFT_SHOULDER])


def attack_right():
    swing([RIGHT_SHOULDER])


def attack():
    """양팔 동시 공격."""
    swing(SHOULDERS)


def attack_combo():
    """왼팔 - 오른팔 - 양팔."""
    attack_left()
    attack_right()
    attack()


# --- 손 흔들기 (인사) ---------------------------------------------

def wave(channels, times=None):
    """손을 좌우로 흔든다.

    먼저 WAVE_LIFT 만큼 손을 들어올리고, 그 자리에서 좌우로 흔든 뒤,
    같은 시간만큼 되내려 원래 자세로 돌아온다.

    CR 서보는 위치 개념이 없어서 '어디까지 올려라'를 시킬 수 없다.
    올린 시간만큼 정확히 되내려야만 제자리로 온다. 그래서 흔드는 구간은
    +WAVE_TIME 과 -WAVE_TIME 이 짝을 이뤄 합이 0 이 되게 짰다.
    """
    n = WAVE_COUNT if times is None else times
    power = pct_to_offset(WAVE_SPEED) * WAVE_FLIP

    for ch in channels:                     # 손 들기
        move_servo(ch, power)
    time.sleep(WAVE_LIFT)

    for _ in range(n):                      # 그 자리에서 흔들기
        for ch in channels:
            move_servo(ch, -power)
        time.sleep(WAVE_TIME)
        for ch in channels:
            move_servo(ch, power)
        time.sleep(WAVE_TIME)

    for ch in channels:                     # 들었던 만큼 되내리기
        move_servo(ch, -power)
    time.sleep(WAVE_LIFT)

    stop_channels(channels)


def wave_left():
    wave([LEFT_WRIST])


def wave_right():
    wave([RIGHT_WRIST])


def wave_both():
    wave(WRISTS)


# --- CLI ----------------------------------------------------------
# 서버(ropi_robot_server.py)가 이 이름들로 호출한다. 바꾸지 말 것.
COMMANDS = {
    "walk": forward,          # 서버 호환: walk = 전진
    "forward": forward,
    "back": backward,
    "left": arc_left,
    "right": arc_right,
    "spinleft": spin_left,
    "spinright": spin_right,
    "stop": stop,
    "home": home,
    "attack": attack,
    "attackleft": attack_left,
    "attackright": attack_right,
    "combo": attack_combo,
    "wave": wave_both,
    "waveleft": wave_left,
    "waveright": wave_right,
    "spindance": spin_dance,
    "dance": robot_dance,
}


def main():
    load_settings()

    if len(sys.argv) < 2:
        print("Usage: python3 ropi_motion.py <command> [value]")
        print("  주행: forward(walk) back left right spinleft spinright stop")
        print("  공격: attack attackleft attackright combo")
        print("  인사: wave waveleft waveright")
        print("  설정: speed <20~100>")
        print("        channels <왼어깨> <오른어깨> <왼손> <오른손>")
        return

    command = sys.argv[1].lower()

    if command == "channels":
        # 팔이 꽂힌 채널이 로봇마다 다르다. 코드를 고치지 않고 여기서 바꾼다.
        #   python3 ropi_motion.py channels 8 9 10 11
        #   순서: 왼어깨 오른어깨 왼손 오른손
        if len(sys.argv) < 6:
            print("현재 채널:")
            print("  바퀴   왼 CH%d  오른 CH%d" % (LEFT_WHEEL, RIGHT_WHEEL))
            print("  어깨   왼 CH%d  오른 CH%d" % (LEFT_SHOULDER, RIGHT_SHOULDER))
            print("  손     왼 CH%d  오른 CH%d" % (LEFT_WRIST, RIGHT_WRIST))
            print("바꾸려면: channels <왼어깨> <오른어깨> <왼손> <오른손>")
            return
        try:
            values = [int(v) for v in sys.argv[2:6]]
        except ValueError:
            print("채널은 숫자여야 합니다:", sys.argv[2:6])
            return
        if any(v < 0 or v > 15 for v in values):
            print("채널은 0~15 사이여야 합니다:", values)
            return
        if len(set(values)) != 4:
            print("채널이 겹칩니다:", values)
            return
        g = globals()
        for key, value in zip(("LEFT_SHOULDER", "RIGHT_SHOULDER",
                               "LEFT_WRIST", "RIGHT_WRIST"), values):
            g[key] = value
        rebuild()
        save_settings()
        print("어깨 CH%d,CH%d / 손 CH%d,CH%d 로 저장했습니다" % tuple(values))
        return

    if command == "speed":
        if len(sys.argv) < 3:
            print("speed:", SPEED)
            return
        try:
            value = int(sys.argv[2])
        except ValueError:
            print("speed 는 숫자여야 합니다:", sys.argv[2])
            return
        set_speed(value)
        save_settings()
        print("speed:", SPEED)
        return

    action = COMMANDS.get(command)
    if action is None:
        print("Unknown command:", command)
        return
    action()


if __name__ == "__main__":
    main()
else:
    load_settings()
