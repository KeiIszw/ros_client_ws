import time
from decimal import Decimal
import pygame
import roslibpy
import math


# ------------------ 定数定義 ------------------
# 建機名
EXCAVATOR_NAME = 'tb20e'

# トピック名
TOPIC_NAME_DRIVE = '/' + EXCAVATOR_NAME + '/tracks/cmd_vel'
TOPIC_NAME_BLADE = '/' + EXCAVATOR_NAME + '/blade/cmd'
TOPIC_NAME_BODY = '/' + EXCAVATOR_NAME + '/body/cmd'
TOPIC_NAME_SWING = '/' + EXCAVATOR_NAME + '/swing/cmd'
TOPIC_NAME_BOOM = '/' + EXCAVATOR_NAME + '/boom/cmd'
TOPIC_NAME_ARM = '/' + EXCAVATOR_NAME + '/arm/cmd'
TOPIC_NAME_BUCKET = '/' + EXCAVATOR_NAME + '/bucket/cmd'
TOPIC_NAME_THUMB = '/' + EXCAVATOR_NAME + '/thumb/cmd'

# トピック型
TOPIC_TYPE_DRIVE = 'geometry_msgs/msg/Twist'
TOPIC_TYPE_BLADE = 'std_msgs/msg/Float64'
TOPIC_TYPE_BODY = 'std_msgs/msg/Float64'
TOPIC_TYPE_SWING = 'std_msgs/msg/Float64'
TOPIC_TYPE_BOOM = 'std_msgs/msg/Float64'
TOPIC_TYPE_ARM = 'std_msgs/msg/Float64'
TOPIC_TYPE_BUCKET = 'std_msgs/msg/Float64'
TOPIC_TYPE_THUMB = 'std_msgs/msg/Float64'

# 制御対象名とトピック名の辞書
TOPIC_NAME_DICT = {
    'drive': TOPIC_NAME_DRIVE,
    'blade': TOPIC_NAME_BLADE,
    'body': TOPIC_NAME_BODY,
    'swing': TOPIC_NAME_SWING,
    'boom': TOPIC_NAME_BOOM,
    'arm': TOPIC_NAME_ARM,
    'bucket': TOPIC_NAME_BUCKET,
    'thumb': TOPIC_NAME_THUMB
}

# 制御対象名とトピック型の辞書
TOPIC_TYPE_DICT = {
    'drive': TOPIC_TYPE_DRIVE,
    'blade': TOPIC_TYPE_BLADE,
    'body': TOPIC_TYPE_BODY,
    'swing': TOPIC_TYPE_SWING,
    'boom': TOPIC_TYPE_BOOM,
    'arm': TOPIC_TYPE_ARM,
    'bucket': TOPIC_TYPE_BUCKET,
    'thumb': TOPIC_TYPE_THUMB
}

# ジョイスティックのマッピング
# procontroller
JOY_MAP = {
    'JOY_BTN_MAP' : {
        'A' : 0,
        'B' : 1,
        'X' : 2,
        'Y' : 3,
        '-' : 4,
        'HOME' : 5,
        '+' : 6,
        'LEFT_STICK_BTN' : 7,
        'RIGHT_STICK_BTN' : 8,
        'L' : 9,
        'R' : 10,
        'UP' : 11,
        'DOWN' : 12,
        'LEFT' : 13,
        'RIGHT' : 14,
        'CAPTURE' : 15
    },
    'JOY_AXE_MAP' : {
        'LEFT_STICK_X' : 0,
        'LEFT_STICK_Y' : 1,
        'RIGHT_STICK_X' : 2,
        'RIGHT_STICK_Y' : 3,
        'ZL' : 4,
        'ZR' : 5
    },
    'JOY_HAT_MAP' : {}
}

# ジョイスティック入力と重機制御の割当
JOY_CTRL_ASSIN = {
    'drive_l+' : 'ZL',
    'drive_r+' : 'ZR',
    'drive_l-' : 'L',
    'drive_r-' : 'R',
    'blade+' : 'UP',
    'blade-' : 'DOWN',
    'body' : 'LEFT_STICK_X',
    'swing+' : 'RIGHT',
    'swing-' : 'LEFT',
    'boom' : 'RIGHT_STICK_Y',
    'arm' : 'LEFT_STICK_Y',
    'bucket' : 'RIGHT_STICK_X',
    'thumb+' : 'A',
    'thumb-' : 'X'
}

# 可動域の最小角度
BLADE_MIN_DEG = -30
SWING_MIN_DEG = -49
BOOM_MIN_DEG = -130
ARM_MIN_DEG = 0
BUCKET_MIN_DEG = -70
THUMB_MIN_DEG = 0

# 可動域の最大角度
BLADE_MAX_DEG = 15
SWING_MAX_DEG = 78
BOOM_MAX_DEG = 0
ARM_MAX_DEG = 120
BUCKET_MAX_DEG = 100
THUMB_MAX_DEG = 140

# 動作速度
DRIVE_SPEED_LINEAR = 1
DRIVE_SPEED_ANGULAR = 1
BLADE_SPEED = 1
BODY_SPEED = 1
SWING_SPEED = 1
BOOM_SPEED = 1
ARM_SPEED = -1
BUCKET_SPEED = 1
THUMB_SPEED = 1

JOY_AXE_THRESHOLD = 0.3 # アナログスティック入力のしきい値

UPDATE_INTERVAL = 0.02  # ジョイスティックの状態更新間隔[s]
HOST = 'localhost'  # ROS2ブリッジのホスト名またはIPアドレス
PORT = 9090          # ROS2ブリッジのポート番号
# ---------------------------------------------


class JoystickPublishManager:
    """
    ジョイスティックの入力から各種コマンドをpublishするクラス
    """
    def __init__(self):
        # joystick初期化
        pygame.init()
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.n_btn = self.joy.get_numbuttons()
        self.n_axe = self.joy.get_numaxes()
        self.n_hat = self.joy.get_numhats()
        print("Joystick Name: " + self.joy.get_name())
        print("Number of Button : " + str(self.n_btn))
        print("Number of Axis : " + str(self.n_axe))
        print("Number of Hats : " + str(self.n_hat))

        pygame.event.get()
        self.s_btn = [0] * self.n_btn
        self.s_axe = [0.0] * self.n_axe
        self.s_hat = [0] * self.n_hat

        # ROS2クライアント初期化
        self.client = roslibpy.Ros(host=HOST, port=PORT)
        self.client.run()

        # トピックパブリッシャのディクショナリ作成
        # {topic_name : publisher}
        self.publishers = {
            name: roslibpy.Topic(self.client, TOPIC_NAME_DICT[name], TOPIC_TYPE_DICT[name])
            for name in TOPIC_NAME_DICT.keys()
        }

        # パブリッシュする角度データの初期化
        self.blade_deg = 0
        self.body_deg = 0
        self.swing_deg = 0
        self.boom_deg = 0
        self.arm_deg = 0
        self.bucket_deg = BUCKET_MIN_DEG
        self.thumb_deg = 0
    

    def update(self):
        """
        ジョイスティックの状態を更新し，各種コマンドをpublishするメソッド
        """
        pygame.event.get()
        # Buttons
        for i in range(self.n_btn):
            self.s_btn[i] = self.joy.get_button(i)
        # Axes
        for i in range(self.n_axe):
            self.s_axe[i] = float(Decimal(self.joy.get_axis(i)).quantize(Decimal('0.01')))
            if abs(self.s_axe[i]) < JOY_AXE_THRESHOLD: # デッドゾーンを設定
                self.s_axe[i] = 0
        # Hats
        for i in range(self.n_hat):
            self.s_hat[i] = self.joy.get_hat(i)
        time.sleep(UPDATE_INTERVAL)

        # デバッグ用
        # print("Btn", end="")
        # print(self.s_btn, end="")
        # print("  Axe", end="")
        # print(self.s_axe, end="")
        # print("  Hat", end="")
        # print(self.s_hat)

        self._cmd_publish()

    def _cmd_publish(self):
        """
        ジョイスティックの状態に基づいて各種コマンドをpublishする
        """
        for name, publisher in self.publishers.items():

            # クローラ
            if name == 'drive':
                # 左プラス
                axe_left_plus = JOY_CTRL_ASSIN['drive_l+']
                axe_left_plus_key = JOY_MAP['JOY_AXE_MAP'][axe_left_plus]
                # ZLは入力なしが-1, 入力ありが1なので, 0 or 1に直す
                left_plus_val = 0
                if self.s_axe[axe_left_plus_key] == 1:
                    left_plus_val = 1
                # 右プラス
                axe_right_plus = JOY_CTRL_ASSIN['drive_r+']
                axe_right_plus_key = JOY_MAP['JOY_AXE_MAP'][axe_right_plus]
                # ZRは入力なしが-1, 入力ありが1なので, 0 or 1に直す
                right_plus_val = 0
                if self.s_axe[axe_right_plus_key] == 1:
                    right_plus_val = 1
                # 左マイナス
                btn_left_minus = JOY_CTRL_ASSIN['drive_l-']
                btn_left_minus_key = JOY_MAP['JOY_BTN_MAP'][btn_left_minus]
                left_minus_val = self.s_btn[btn_left_minus_key]
                # 右マイナス
                btn_right_minus = JOY_CTRL_ASSIN['drive_r-']
                btn_right_minus_key = JOY_MAP['JOY_BTN_MAP'][btn_right_minus]
                right_minus_val = self.s_btn[btn_right_minus_key]
                
                # 移動の処理
                v = 0 # 速度
                w = 0 # 角速度
                # 停止
                if left_plus_val == 1 and left_minus_val == 1:
                    v = 0
                    w = 0
                # 停止
                elif right_plus_val == 1 and right_minus_val == 1:
                    v = 0
                    w = 0
                # 前進
                elif left_plus_val == 1 and right_plus_val == 1:
                    v = DRIVE_SPEED_LINEAR
                    w = 0
                # 後退
                elif left_minus_val == 1 and right_minus_val ==1:
                    v = -DRIVE_SPEED_LINEAR
                    w = 0
                # 右旋回
                elif left_plus_val == 1 and right_minus_val == 1:
                    v = 0
                    w = -DRIVE_SPEED_ANGULAR
                # 左旋回
                elif left_minus_val == 1 and right_plus_val == 1:
                    v = 0
                    w = DRIVE_SPEED_ANGULAR
                # 右方前進
                elif left_plus_val == 1:
                    v = DRIVE_SPEED_LINEAR / 2
                    w = -DRIVE_SPEED_ANGULAR
                # 左方前進
                elif right_plus_val == 1:
                    v = DRIVE_SPEED_LINEAR / 2
                    w = DRIVE_SPEED_ANGULAR
                # 右方後退
                elif left_minus_val == 1:
                    v = -DRIVE_SPEED_LINEAR / 2
                    w = DRIVE_SPEED_ANGULAR
                # 左方後退
                elif right_minus_val == 1:
                    v = -DRIVE_SPEED_LINEAR / 2
                    w = -DRIVE_SPEED_ANGULAR

                # publish
                msg = roslibpy.Message({'linear': {'x': v, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': w}})
                publisher.publish(msg) # publish
            # ブレード
            elif name == 'blade':
                # プラスボタン
                btn_plus = JOY_CTRL_ASSIN['blade+'] # ボタン名
                btn_plus_key = JOY_MAP['JOY_BTN_MAP'][btn_plus] # ボタンキー
                # マイナスボタン
                btn_minus = JOY_CTRL_ASSIN['blade-'] # ボタン名
                btn_minus_key = JOY_MAP['JOY_BTN_MAP'][btn_minus] # ボタンキー
                self.blade_deg += (self.s_btn[btn_plus_key] - self.s_btn[btn_minus_key]) * BLADE_SPEED # 入力値をもとに角度を更新
                # 可動域の制限
                if self.blade_deg <= BLADE_MIN_DEG:
                    self.blade_deg = BLADE_MIN_DEG
                elif self.blade_deg >= BLADE_MAX_DEG:
                    self.blade_deg = BLADE_MAX_DEG
                # degをradに変換してpublish
                msg = roslibpy.Message({'data': math.radians(self.blade_deg)})
                publisher.publish(msg)
            # ボディ
            elif name == 'body':
                axe = JOY_CTRL_ASSIN['body'] # 軸名
                axe_key = JOY_MAP['JOY_AXE_MAP'][axe] # 軸キー
                self.body_deg += self.s_axe[axe_key] * BODY_SPEED
                # publish
                msg = roslibpy.Message({'data': math.radians(self.body_deg)})
                publisher.publish(msg)
            # スイング
            elif name == 'swing':
                btn_plus = JOY_CTRL_ASSIN['swing+']
                btn_plus_key = JOY_MAP['JOY_BTN_MAP'][btn_plus]
                btn_minus = JOY_CTRL_ASSIN['swing-']
                btn_minus_key = JOY_MAP['JOY_BTN_MAP'][btn_minus]
                self.swing_deg += (self.s_btn[btn_plus_key] - self.s_btn[btn_minus_key]) * SWING_SPEED
                # 可動域の制限
                if self.swing_deg <= SWING_MIN_DEG:
                    self.swing_deg = SWING_MIN_DEG
                elif self.swing_deg >= SWING_MAX_DEG:
                    self.swing_deg = SWING_MAX_DEG
                # publish
                msg = roslibpy.Message({'data': math.radians(self.swing_deg)})
                publisher.publish(msg)
            # ブーム
            elif name == 'boom':
                axe = JOY_CTRL_ASSIN['boom']
                axe_key = JOY_MAP['JOY_AXE_MAP'][axe]
                self.boom_deg += self.s_axe[axe_key] * BOOM_SPEED
                # 可動域の制限
                if self.boom_deg <= BOOM_MIN_DEG:
                    self.boom_deg = BOOM_MIN_DEG
                elif self.boom_deg >= BOOM_MAX_DEG:
                    self.boom_deg = BOOM_MAX_DEG
                # publish
                msg = roslibpy.Message({'data': math.radians(self.boom_deg)})
                publisher.publish(msg)
            # アーム
            elif name == 'arm':
                axe = JOY_CTRL_ASSIN['arm']
                axe_key = JOY_MAP['JOY_AXE_MAP'][axe]
                self.arm_deg += self.s_axe[axe_key] * ARM_SPEED
                # 可動域の制限
                if self.arm_deg <= ARM_MIN_DEG:
                    self.arm_deg = ARM_MIN_DEG
                elif self.arm_deg >= ARM_MAX_DEG:
                    self.arm_deg = ARM_MAX_DEG
                # publish
                msg = roslibpy.Message({'data': math.radians(self.arm_deg)})
                publisher.publish(msg)
            # バケット
            elif name == 'bucket':
                axe = JOY_CTRL_ASSIN['bucket']
                axe_key = JOY_MAP['JOY_AXE_MAP'][axe]
                self.bucket_deg += self.s_axe[axe_key] * BUCKET_SPEED
                # 可動域の制限
                if self.bucket_deg <= BUCKET_MIN_DEG:
                    self.bucket_deg = BUCKET_MIN_DEG
                elif self.bucket_deg >= BUCKET_MAX_DEG:
                    self.bucket_deg = BUCKET_MAX_DEG
                # publish
                msg = roslibpy.Message({'data': math.radians(self.bucket_deg)})
                publisher.publish(msg)
            # サム
            elif name == 'thumb':
                btn_plus = JOY_CTRL_ASSIN['thumb+']
                btn_plus_key = JOY_MAP['JOY_BTN_MAP'][btn_plus]
                btn_minus = JOY_CTRL_ASSIN['thumb-']
                btn_minus_key = JOY_MAP['JOY_BTN_MAP'][btn_minus]
                self.thumb_deg += (self.s_btn[btn_plus_key] - self.s_btn[btn_minus_key]) * THUMB_SPEED
                # 可動域の制限
                if self.thumb_deg <= THUMB_MIN_DEG:
                    self.thumb_deg = THUMB_MIN_DEG
                elif self.thumb_deg >= THUMB_MAX_DEG:
                    self.thumb_deg = THUMB_MAX_DEG
                # publish
                msg = roslibpy.Message({'data': math.radians(self.thumb_deg)})
                publisher.publish(msg)


def main():
    joystick_manager = JoystickPublishManager()
    
    try:
        while True:
            joystick_manager.update()
    except(KeyboardInterrupt, SystemExit):  # Exit with Ctrl-C
        print("Exit")

        # publish終了処理
        for publisher in joystick_manager.publishers.values():
            publisher.unadvertise()
        joystick_manager.client.terminate()


if __name__ == "__main__":
    main()