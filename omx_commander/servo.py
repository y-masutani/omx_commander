import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist, TwistStamped
from std_srvs.srv import Trigger
import time
import threading
from omx_commander.kbhit import KBHit
from pymoveit2 import MoveIt2, GripperInterface

GRIPPER_MIN = -0.010
GRIPPER_MAX = 0.019

# OMX用のservo_nodeへ指令を送るノード
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
        self.publisher_joint_jog = self.create_publisher(
            JointJog,
            'servo_node/delta_joint_cmds', 10)
        self.publisher_twist = self.create_publisher(
            TwistStamped,
            'servo_node/delta_twist_cmds', 10)
        self.client_start_servo = self.create_client(
            Trigger, 'servo_node/start_servo')
        self.client_stop_servo = self.create_client(
            Trigger, 'servo_node/stop_servo')

    def publish_twist(self, frame_id, twist: Twist):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.twist = twist
        self.publisher_twist.publish(msg)

    def publish_joint_jog(self, names, velocities):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'link1'
        msg.joint_names = names
        msg.velocities = velocities
        self.publisher_joint_jog.publish(msg)

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
    
    def start_moveit_servo(self):
        while not self.client_start_servo.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービス無効．待機中…')
        future = self.client_start_servo.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def stop_moveit_servo(self):
        while not self.client_stop_servo.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービス無効．待機中…')
        future = self.client_stop_servo.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def add_collision(self):
        self.moveit2.add_collision_box(
            id='floor_top', size=[1.0, 1.0, 0.002],
            position=[0.0, 0.0, -0.001], quat_xyzw=[0.0, 0.0, 0.0, 1.0])

    def remove_collision(self):
        self.moveit2.remove_collision_object('floor_top')


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

    # 最初の指令をパブリッシュする前に少し待つ
    time.sleep(1.0)

    # 初期ポーズへゆっくり移動させる
    joint = [0.0, 0.0, 0.0, 0.0]
    commander.set_max_velocity(0.2)
    commander.move_joint(joint)
 
    # 障害物の追加
    commander.add_collision()

    kb = KBHit()

    # サーボの機能を有効化
    commander.start_moveit_servo()

    print('1, 2, 3, 4, 5, 6, 7, 8, 9, 0キーを押して関節を動かす')
    print('a, z, s, x, d, c, f, vキーを押して手先を動かす')
    print('スペースキーを押して初期ポーズにする')
    print('wキーを押して土台の座標系を基準とする')
    print('eキーを押して手先の座標系を基準とする')
    print('oキーを押して障害物を追加する')
    print('Oキーを押して障害物を追加する')
    print('Escキーを押して終了')

    frame_id = 'link1'
    gripper = 0.0

    # Ctrl+CでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        while True:
            time.sleep(0.01)
            # キーが押されているか？
            if kb.kbhit():
                c = kb.getch()
                name = None
                velocity = 0
                twist = Twist()
                # 押されたキーによって場合分けして処理
                if c == '1':
                    name, velocity = 'joint1', -3.0
                elif c == '2':
                    name, velocity = 'joint1', +3.0
                elif c == '3':
                    name, velocity = 'joint2', -3.0
                elif c == '4':
                    name, velocity = 'joint2', +3.0
                elif c == '5':
                    name, velocity = 'joint3', -3.0
                elif c == '6':
                    name, velocity = 'joint3', +3.0
                elif c == '7':
                    name, velocity = 'joint4', -3.0
                elif c == '8':
                    name, velocity = 'joint4', +3.0
                elif c == '9':
                    gripper -= 0.001
                elif c == '0':
                    gripper += 0.001
                elif c == 'a':
                    twist.linear.x = +1.0
                elif c == 'z':
                    twist.linear.x = -1.0
                elif c == 's':
                    twist.linear.y = +1.0
                elif c == 'x':
                    twist.linear.y = -1.0
                elif c == 'd':
                    twist.linear.z = +1.0
                elif c == 'c':
                    twist.linear.z = -1.0
                elif c == 'f':
                    twist.angular.y = +1.0
                elif c == 'v':
                    twist.angular.y = -1.0
                elif c == 'w':
                    frame_id = 'link1'
                elif c == 'e':
                    frame_id = 'end_effector_link'

                if ord(c) == 27:  # Escキー
                    break
                elif c == ' ':  # スペースキー
                    joint = [0.0, 0.0, 0.0, 0.0]
                    commander.move_joint(joint)
                    continue
                elif c in '12345678':
                    commander.publish_joint_jog([name], [velocity])
                elif c in '90':
                    commander.move_gripper(gripper)
                elif c in 'azsxdcfv':
                    commander.publish_twist(frame_id, twist)
                elif c in 'we':
                    print(f'{frame_id=}')
                elif c == 'o':
                    print('障害物を追加')
                    commander.add_collision()
                elif c == 'O':
                    print('障害物を削除')
                    commander.remove_collision()

    except KeyboardInterrupt:
        thread.join()
    else:
        print('終了')
        # サーボの機能を無効化
        commander.stop_moveit_servo()
        # 終了ポーズへゆっくり移動させる
        joint = [0.00, 0.85, 0.43, -1.23]
        gripper = 0
        commander.set_max_velocity(0.2)
        commander.move_joint(joint)

    rclpy.try_shutdown()
