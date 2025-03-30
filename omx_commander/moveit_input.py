import threading
from math import atan2
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import quaternion_from_euler
from pymoveit2 import MoveIt2, GripperInterface
import readline # input()に入力履歴・行編集の機能を追加するために必要

GRIPPER_MIN = -0.010
GRIPPER_MAX = 0.019


# OMX用のMoveItで関節や手先を位置決めするノード
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
        ]
        callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name='link1',
            end_effector_name='end_effector_link',
            group_name='arm',
            callback_group=callback_group,
        )
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0

        gripper_joint_names = ['gripper_left_joint']
        self.gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=[GRIPPER_MIN],
            closed_gripper_joint_positions=[GRIPPER_MAX],
            gripper_group_name='gripper',
            callback_group=callback_group,
        )
        self.gripper_interface.max_velocity = 1.0
        self.gripper_interface.max_acceleration = 1.0

    def move_joint(self, q):
        joint_positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        self.moveit2.move_to_configuration(joint_positions)
        return self.moveit2.wait_until_executed()

    def move_gripper(self, q):
        position = float(q)
        self.gripper_interface.move_to_position(position)
        return self.gripper_interface.wait_until_executed()

    def set_max_velocity(self, v):
        self.moveit2.max_velocity = float(v)

    def move_initial(self):
        joint = [0.0, 0.0, 0.0, 0.0]
        self.set_max_velocity(0.2)
        self.move_joint(joint)
        self.move_gripper(GRIPPER_MIN)

    def move_final(self):
        joint = [0.00, 0.85, 0.43, -1.23]  # 手先を下ろした姿勢 
        self.set_max_velocity(0.2)
        self.move_joint(joint)
        self.move_gripper(GRIPPER_MAX)

    def move_endtip(self, endtip):
        position = [float(endtip[0]), float(endtip[1]), float(endtip[2])]
        yaw = atan2(position[1], position[0] - 0.012)
        pitch = float(endtip[3])
        quat_xyzw = quaternion_from_euler(0.0, pitch, yaw)
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw
        )
        return self.moveit2.wait_until_executed()


def print_help():
    print(
        '以下のいずれかのコマンドを入力してEnterキーを押して実行\n' +
        '関節     → joint 関節1値 関節2値 関節3値 関節4値\n' +
        '手先     → endtip x座標 y座標 z座標 ピッチ角\n' +
        'グリッパ → gripper グリッパ値\n' +
        '初期姿勢 → zero\n' +
        '終了     → quit\n' +
        '（↑↓キーで入力履歴呼出，←→キーで行編集，Enterキーで再実行）'
    )

def main():
    # ROSクライアントの初期化
    rclpy.init()
    # ノードクラスのインスタンス
    commander = Commander()
    # 別のスレッドでrclpy.spin()を実行する
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(commander,executor,))
    threading.excepthook = lambda x: ()
    thread.start()
    # 初期ポーズへゆっくり移動させる
    commander.move_initial()

    print_help()

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            line = input('コマンド: ')
            word = line.split()
            if word == []:
                print_help()
                continue
            if word[0] == 'quit':
                break
            elif word[0] == 'zero':
                commander.move_initial()
            elif word[0] == 'joint':
                if len(word) < 5:
                    print('引数が少なすぎる')
                    continue
                try:
                    joint = [float(x) for x in word[1:5]]
                except ValueError:
                    print('引数が数値として解釈できない')
                    continue
                r = commander.move_joint(joint)
                print(f'戻り値： {r}')
            elif word[0] == 'endtip':
                if len(word) < 5:
                    print('引数が少なすぎる')
                    continue
                try:
                    endtip = [float(x) for x in word[1:5]]
                except ValueError:
                    print('引数が数値として解釈できない')
                    continue
                r = commander.move_endtip(endtip)
                print(f'戻り値： {r}')
            elif word[0] == 'gripper':
                if len(word) < 2:
                    print('引数が少なすぎる')
                    continue
                try:
                    gripper = float(word[1])
                except ValueError:
                    print('引数が数値として解釈できない')
                    continue
                r = commander.move_gripper(gripper)
                print(f'戻り値： {r}')
            else:
                print(f'{word[0]}はコマンドとして解釈できない')
    except KeyboardInterrupt:
        thread.join()
    else:
        print('終了')
        # 終了ポーズへゆっくり移動させる
        commander.move_final()

    rclpy.try_shutdown()
