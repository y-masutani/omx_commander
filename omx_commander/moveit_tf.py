import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
import threading
from math import sqrt, atan2
from omx_commander.kbhit import KBHit
from pymoveit2 import MoveIt2, GripperInterface
import readline # input()に入力履歴・行編集の機能を追加するために必要
from tf_transformations import euler_from_quaternion, quaternion_from_euler

GRIPPER_MIN = -0.010
GRIPPER_MAX = 0.019

# OMX用のMoveItで逆運動学を計算しtfのフレームで与えられた点へ手先を位置決めするノード
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

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.move_gripper( GRIPPER_MIN)

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

    def get_frame_position(self, frame_id):
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'link1',
                frame_id,
                when,
                timeout=Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(f'{ex}')
            return None
        t = trans.transform.translation
        r = trans.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])
        return [t.x, t.y, t.z, roll, pitch, yaw]


# リストで表された3次元座標間の距離を計算する
def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

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

    # キー読み取りクラスのインスタンス
    kb = KBHit()

    # 状態
    INIT = 0
    WAIT = 1
    DONE = 2
    state = INIT

    print('rキーを押して再初期化')
    print('Escキーを押して終了')

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            time.sleep(0.01)
            # キーが押されているか？
            if kb.kbhit():
                c = kb.getch()
                if c == 'r':
                    print('再初期化')
                    state = INIT
                elif ord(c) == 27:  # Escキー
                    break

            position = commander.get_frame_position('target')
            if position is None:
                print('対象のフレームが見つからない')
            else:
                xyz_now = position[0:3]
                time_now = time.time()
                if state == INIT:
                    xyz_first = xyz_now
                    time_first = time_now
                    state = WAIT
                elif state == WAIT:
                    if dist(xyz_now, xyz_first) > 0.01:
                        state = INIT
                    elif time_now - time_first > 1.0:
                        state = DONE
                        commander.move_gripper(GRIPPER_MAX)
                        commander.set_max_velocity(1.0)
                        pitch = 0
                        sucess = commander.move_endtip(xyz_now + [pitch])
                        if sucess:
                            print('move_endtip()成功')
                        else:
                            print('move_endtip()失敗')
                        commander.move_gripper(GRIPPER_MIN)
    except KeyboardInterrupt:
        thread.join()
    else:
        print('終了')
        # 終了ポーズへゆっくり移動させる
        commander.move_final()

    rclpy.try_shutdown()
