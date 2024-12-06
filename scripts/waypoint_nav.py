#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformListener, Buffer
import yaml
import math
import threading
import argparse  # argparseを使用
import time


class Nav2Client(Node):
    def __init__(self, waypoints):
        super().__init__('nav2_send_goal_python')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.started = False
        self.restart_pending = False
        self.stop_restart_message = False  # リスタートメッセージ停止フラグ
        self.stop_goal_message = False  # ゴールメッセージ停止フラグ
        self.timer = self.create_timer(0.5, self.check_distance_and_update_goal)

        # QoSプロファイルを作成
        qos_profile = QoSProfile(
            depth=10,  # キューの深さ
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # パブリッシャがメッセージを保持
            reliability=QoSReliabilityPolicy.RELIABLE  # 確実なメッセージ配信
        )

        # トピックのサブスクライバとパブリッシャを作成
        self.trigger_subscriber = self.create_subscription(
            String,
            'waypoint_cmd_trigger',
            self.trigger_callback,
            10
        )
        self.speak_publisher = self.create_publisher(String, '/speak', qos_profile)

        self.get_logger().info('Waiting for start command on topic "waypoint_cmd_trigger"...')

    def trigger_callback(self, msg):
        if msg.data == 's' and not self.started:
            self.started = True
            self.get_logger().info('Received start command, starting navigation...')
            self.publish_speak_message('ナビゲーションを開始します')
            self.send_goal(self.waypoints[self.current_waypoint_index])
        elif msg.data == 'r' and self.restart_pending:
            self.stop_restart_message = True  # リスタートメッセージ送信停止
            self.restart_pending = False  # リスタート待ち状態を解除
            self.get_logger().info('Restart command received, moving to the next waypoint...')
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.send_goal(self.waypoints[self.current_waypoint_index])
            else:
                self.start_goal_message_publishing()

    def is_restart_mode(self):
        """現在のウェイポイントがリスタートモードかを確認"""
        current_waypoint = self.waypoints[self.current_waypoint_index]
        return any(func['function'] == 'restart' for func in current_waypoint.get('functions', []))

    def send_goal(self, waypoint):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = waypoint['position']['x']
        goal_msg.pose.pose.position.y = waypoint['position']['y']
        goal_msg.pose.pose.orientation.z = waypoint['euler_angle']['z']
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()

        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Goal succeeded!')

    def check_distance_and_update_goal(self):
        if not self.started:
            return

        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y

            waypoint = self.waypoints[self.current_waypoint_index]
            distance = math.sqrt(
                (waypoint['position']['x'] - current_x) ** 2 +
                (waypoint['position']['y'] - current_y) ** 2
            )

            waypoint_radius = next(
                (func['waypoint_radius'] for func in waypoint.get('functions', []) if func['function'] == 'variable_waypoint_radius'),
                1.0  
            )

            self.get_logger().info(f'Distance to waypoint {self.current_waypoint_index + 1}: {distance} (tolerance: {waypoint_radius})')

            if distance < waypoint_radius:
                if self.is_restart_mode() and not self.restart_pending:
                    self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached (Restart mode).')
                    self.restart_pending = True  # リスタート待ち状態に設定
                    self.stop_restart_message = False  # 停止フラグをリセット
                    threading.Thread(target=self.publish_restart_message).start()
                elif not self.is_restart_mode():
                    self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached!')
                    self.publish_speak_message('ウェイポイントを通過しました')
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index < len(self.waypoints):
                        self.send_goal(self.waypoints[self.current_waypoint_index])
                    else:
                        self.start_goal_message_publishing()

        except Exception as e:
            self.get_logger().warn(f'Error while checking distance: {e}')

    def publish_restart_message(self):
        """リスタートモード時にリスタート可能メッセージを3秒間隔で送信"""
        while not self.stop_restart_message:
            self.publish_speak_message('リスタートできます')
            time.sleep(3)  # 3秒間隔で送信

    def start_goal_message_publishing(self):
        """ゴール後のメッセージ送信を開始"""
        self.get_logger().info('Final waypoint reached, starting goal message publishing.')
        self.stop_goal_message = False  # 停止フラグをリセット
        threading.Thread(target=self.publish_goal_message).start()

    def publish_goal_message(self):
        """ゴール後のメッセージを5秒間隔で送信"""
        while not self.stop_goal_message:
            self.publish_speak_message('ゴールしました')
            time.sleep(5)  # 5秒間隔で送信

    def publish_speak_message(self, message):
        """/speakトピックにメッセージを送信する"""
        msg = String()
        msg.data = message
        self.speak_publisher.publish(msg)


def load_waypoints_from_yaml(file_path):
    with open(file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    return yaml_data['waypoints']


def main(args=None):
    rclpy.init(args=args)

    # argparseを使用して引数を解析
    parser = argparse.ArgumentParser(description='Nav2Client Node')

    # 短い形式と長い形式の引数を定義
    parser.add_argument(
        '-w', '--waypoint_yaml_path',
        required=True,
        help='Path to the YAML file containing waypoints'
    )

    # 未知の引数を無視して解析
    parsed_args, unknown_args = parser.parse_known_args()

    # YAMLパスの取得
    yaml_file_path = parsed_args.waypoint_yaml_path
    waypoints = load_waypoints_from_yaml(yaml_file_path)

    nav_client = Nav2Client(waypoints)

    rclpy.spin(nav_client)

    rclpy.shutdown()


if __name__ == '__main__':
    main()