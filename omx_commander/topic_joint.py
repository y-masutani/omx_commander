import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from omx_commander.kbhit import KBHit

GRIPPER_MIN = -0.010
GRIPPER_MAX = 0.019


# OMX用のトピックへ指令をパブリッシュするノード
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
        ]
        self.publisher_joint = self.create_publisher(
            JointTrajectory,
            'arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(
            self, GripperCommand, 'gripper_controller/gripper_cmd'
        )

    def publish_joint(self, q, time):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_joint.publish(msg)

    def send_gripper_command(self, g):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(g)
        goal_msg.command.max_effort = 10.0
        self.gripper_client.wait_for_server()
        return self.gripper_client.send_goal(goal_msg)
    def move_initial(self):
        joint = [0.0, 0.0, 0.0, 0.0]
        dt = 5
        self.publish_joint(joint, dt)
        self.send_gripper_command(GRIPPER_MIN)

    def move_final(self):
        joint = [0.00, 0.85, 0.43, -1.23]  # 手先を下ろした姿勢
        dt = 5
        self.publish_joint(joint, dt)
        self.send_gripper_command(GRIPPER_MAX)

def main():
    # ROSクライアントの初期化
    rclpy.init()
    # ノードクラスのインスタンス
    commander = Commander()
    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    threading.excepthook = lambda x: ()
    thread.start()
    # 初期ポーズへゆっくり移動させる
    commander.move_initial()
   
    # キー読み取りクラスのインスタンス
    kb = KBHit()

    print('1, 2, 3, 4, 5, 6, 7, 8, 9, 0キーを押して関節を動かす')
    print('スペースキーを押して初期ポーズにする')
    print('Escキーを押して終了')

    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = GRIPPER_MIN

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            time.sleep(0.01)
            # キーが押されているか？
            if kb.kbhit():
                c = kb.getch()
                # 変更前の値を保持
                joint_prev = joint.copy()
                gripper_prev = gripper

                # 目標関節値とともに送る目標時間
                dt = 0.2

                # 押されたキーによって場合分けして処理
                if c == '1':
                    joint[0] -= 0.1
                elif c == '2':
                    joint[0] += 0.1
                elif c == '3':
                    joint[1] -= 0.1
                elif c == '4':
                    joint[1] += 0.1
                elif c == '5':
                    joint[2] -= 0.1
                elif c == '6':
                    joint[2] += 0.1
                elif c == '7':
                    joint[3] -= 0.1
                elif c == '8':
                    joint[3] += 0.1
                elif c == '9':
                    gripper -= 0.001
                elif c == '0':
                    gripper += 0.001
                elif c == ' ':  # スペースキー
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    dt = 1.0
                elif ord(c) == 27:  # Escキー
                    break

                # 変化があればパブリッシュ
                publish = False
                if joint != joint_prev:
                    print((f'joint: [{joint[0]:.2f}, {joint[1]:.2f}, '
                           f'{joint[2]:.2f}, {joint[3]:.2f}]'))
                    commander.publish_joint(joint, dt)
                    publish = True
                if gripper != gripper_prev:
                    print(f'gripper: {gripper:.3f}')
                    commander.send_gripper_command(gripper)
                    publish = True
                # パブリッシュした場合は，設定時間と同じだけ停止
                if publish:
                    time.sleep(dt)
    except KeyboardInterrupt:
        thread.join()
    else:
        print('終了')
        # 終了ポーズへゆっくり移動させる
        commander.move_final()

    rclpy.try_shutdown()
