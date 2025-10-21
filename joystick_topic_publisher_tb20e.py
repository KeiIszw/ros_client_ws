# Copyright (c) 2021 Takenoshin
# Released under the MIT license
# https://opensource.org/licenses/mit-license.php
# https://github.com/tak6uch1/joystick

import time
from decimal import Decimal
import pygame
import roslibpy
import math


# ------------------ 定数定義 ------------------
# トピック名
TOPIC_NAME_DRIVE = '/tb20e/tracks/cmd_vel'
TOPIC_NAME_BLADE = '/tb20e/blade/cmd'
TOPIC_NAME_BODY = '/tb20e/body/cmd'
TOPIC_NAME_SWING = '/tb20e/swing/cmd'
TOPIC_NAME_BOOM = '/tb20e/boom/cmd'
TOPIC_NAME_ARM = '/tb20e/arm/cmd'
TOPIC_NAME_BUCKET = '/tb20e/bucket/cmd'
TOPIC_NAME_THUMB = '/tb20e/thumb/cmd'

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
    'swing+' : 'LEFT',
    'swing-' : 'RIGHT',
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
DRIVE_SPEED = 1
BLADE_SPEED = 1
BODY_SPEED = 1
SWING_SPEED = 1
BOOM_SPEED = 1
ARM_SPEED = 1
BUCKET_SPEED = 1
THUMB_SPEED = 1

JOY_AXE_THRESHOLD = 0.2 # アナログスティック入力のしきい値

UPDATE_INTERVAL = 0.1  # ジョイスティックの状態更新間隔[s]
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
            # msg = roslibpy.Message({'data': self.s_axe[JOY_MAP['JOY_AXE_MAP']['LEFT_STICK_X']]}) # 仮 driveはこれじゃだめ
            # publisher.publish(msg)

            # クローラ
            if name == 'drive':
                pass ############### 未実装 ###############
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
                publisher.publish(math.radians(self.blade_deg)) # degをradに変換してpublish
            # ボディ
            elif name == 'body':
                axe = JOY_CTRL_ASSIN['body'] # 軸名
                axe_key = JOY_MAP['JOY_AXE_MAP'][axe] # 軸キー
                self.body_deg += self.s_axe[axe_key] * BODY_SPEED
                # +-360度に直す
                if self.body_deg <= -360:
                    self.body_deg += 360
                elif self.body_deg >= 360:
                    self.blade_deg -= 360
                publisher.publish(math.radians(self.body_deg))
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
                publisher.publish(math.radians(self.swing_deg))
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
                publisher.publish(math.radians(self.boom_deg))
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
                publisher.publish(math.radians(self.arm_deg))
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
                publisher.publish(math.radians(self.bucket_deg))
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
                publisher.publish(math.radians(self.thumb_deg))


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