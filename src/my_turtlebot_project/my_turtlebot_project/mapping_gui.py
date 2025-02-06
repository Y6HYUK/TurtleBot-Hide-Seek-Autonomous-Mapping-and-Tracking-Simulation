#!/usr/bin/env python3
import sys
import threading
import subprocess
import time
import random
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatusArray

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QHBoxLayout
from PyQt5.QtCore import QTimer

# Frontier 탐색 및 초기 위치 설정 노드를 GUI와 통합한 클래스
class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        # 탐색 상태를 기록할 변수 (GUI에서 폴링)
        self.mapping_status = "Idle"
        self.mapping_completed = False  # 매핑 완료 여부
        
        # 구독자 및 발행자 설정
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',  # 실제 환경에 맞게 수정
            self.goal_status_callback,
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_system_default)
        self.goal_publisher = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
        # 초기 위치 발행을 위한 publisher
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.initial_pose_published = False

        self.map_array = None
        self.map_metadata = None
        self.goal_reached = True  # 초기에는 목표 도착 상태로 가정
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.timer = None  # 타이머는 start_exploration()에서 생성
        self.no_frontier_count = 0  # frontier 미발견 횟수 카운터

    def start_exploration(self):
        self.get_logger().info("Starting frontier exploration...")
        self.mapping_status = "Exploration started"
        self.mapping_completed = False
        if not self.initial_pose_published:
            self.publish_initial_pose()
        # 타이머 주기를 1초로 설정 (빈번하게 목표 산출)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 1.0   # 원하는 초기 x 좌표 (조정 가능)
        msg.pose.pose.position.y = 1.0   # 원하는 초기 y 좌표 (조정 가능)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.25, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.25
        ]
        self.initial_pose_pub.publish(msg)
        self.get_logger().info("Published initial pose automatically.")
        self.initial_pose_published = True

    def complete_mapping(self):
        self.get_logger().info("Mapping marked as completed.")
        self.mapping_status = "Mapping completed"
        self.mapping_completed = True
        # 타이머 중단: 더 이상 frontier 탐지를 실행하지 않도록 함.
        if self.timer is not None:
            self.timer.cancel()

    def map_callback(self, msg: OccupancyGrid):
        data = np.array(msg.data, dtype=np.int8)
        self.map_array = data.reshape((msg.info.height, msg.info.width))
        self.map_metadata = msg.info

    def goal_status_callback(self, msg: GoalStatusArray):
        if msg.status_list:
            current_status = msg.status_list[-1].status
            if current_status == 3:
                self.goal_reached = True
                self.mapping_status = "Goal reached"
                self.get_logger().info("Goal reached successfully.")
            elif current_status in [4, 5, 6]:
                self.goal_reached = True
                self.mapping_status = "Goal failed or aborted"
                self.get_logger().info("Goal failed/aborted.")
            else:
                self.goal_reached = False
                self.mapping_status = f"Goal status: {current_status}"

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def timer_callback(self):
        if self.mapping_completed:
            return  # mapping이 완료되었으면 더 이상 진행하지 않음.

        if self.map_array is None or self.map_metadata is None:
            self.get_logger().info("Waiting for map data...")
            self.mapping_status = "Waiting for map data..."
            return

        if not self.goal_reached:
            self.get_logger().info("Currently moving toward a goal; skipping frontier detection.")
            self.mapping_status = "Moving toward goal"
            return

        frontiers = self.detect_frontiers()
        if not frontiers:
            self.get_logger().info("No frontiers detected.")
            self.mapping_status = "No frontiers"
            self.no_frontier_count += 1
            if self.no_frontier_count >= 3:
                self.complete_mapping()
            return
        else:
            self.no_frontier_count = 0

        valid_frontiers = []
        for (col, row) in frontiers:
            if not self.is_near_wall(col, row, self.map_array, 8):
                valid_frontiers.append((col, row))

        if not valid_frontiers:
            self.get_logger().info("No valid frontiers found (all near walls).")
            self.mapping_status = "No valid frontiers"
            self.no_frontier_count += 1
            if self.no_frontier_count >= 3:
                self.complete_mapping()
            return

        goal_xy = self.select_goal(valid_frontiers)
        if goal_xy is None:
            self.get_logger().info("Failed to select a goal.")
            self.mapping_status = "Goal selection failed"
            return

        self.publish_goal(goal_xy)

    def detect_frontiers(self):
        H, W = self.map_array.shape
        frontiers = []
        for row in range(1, H - 1):
            for col in range(1, W - 1):
                if self.map_array[row, col] == -1:
                    neighbors = self.map_array[row - 1:row + 2, col - 1:col + 2]
                    if (neighbors == 0).any():
                        frontiers.append((col, row))
        return frontiers

    def is_near_wall(self, col, row, map_data, threshold):
        H, W = map_data.shape
        for dy in range(-threshold, threshold + 1):
            for dx in range(-threshold, threshold + 1):
                nr = row + dy
                nc = col + dx
                if 0 <= nr < H and 0 <= nc < W:
                    if map_data[nr, nc] == 100:
                        return True
        return False

    def select_goal(self, frontiers):
        if not frontiers:
            return None
        chosen = random.choice(frontiers)
        col, row = chosen
        res = self.map_metadata.resolution
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        real_x = col * res + origin_x
        real_y = row * res + origin_y
        return (real_x, real_y)

    def publish_goal(self, goal):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal: x={goal[0]:.3f}, y={goal[1]:.3f}")
        self.goal_reached = False
        self.mapping_status = f"Moving to goal: ({goal[0]:.2f}, {goal[1]:.2f})"

    def save_map(self):
        self.get_logger().info("Saving map...")
        import os
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory('my_turtlebot_project')
        map_dir = os.path.join(package_share, 'maps')
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
        map_file_prefix = os.path.join(map_dir, "map")
        try:
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_file_prefix],
                check=True
            )
            self.get_logger().info("Map saved successfully in: " + map_dir)
            self.mapping_status = "Mapping completed and map saved"
        except subprocess.CalledProcessError as e:
            self.get_logger().error("Failed to save map: " + str(e))
            self.mapping_status = "Map saving failed"

# GUI 클래스: PyQt5를 이용하여 버튼 클릭 시 탐색 시작, 매핑 완료 표시 및 맵 저장 요청
class MappingGUI(QWidget):
    def __init__(self, mapping_controller: FrontierExplorationNode):
        super().__init__()
        self.mapping_controller = mapping_controller
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("Mapping Controller")
        self.setMinimumSize(500, 250)
        self.setStyleSheet("""
            QWidget {
                background-color: #f7f7f7;
                font-family: Helvetica, Arial, sans-serif;
                font-size: 14px;
            }
            QPushButton {
                background-color: #008CBA;
                color: white;
                border: none;
                padding: 12px 20px;
                border-radius: 6px;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
            QPushButton:pressed {
                background-color: #005f73;
            }
            QLabel {
                color: #333333;
            }
        """)

        main_layout = QVBoxLayout()
        self.status_label = QLabel("Mapping status: Idle")
        main_layout.addWidget(self.status_label)
        
        button_layout = QHBoxLayout()
        self.start_button = QPushButton("Start Mapping")
        self.start_button.clicked.connect(self.start_mapping)
        button_layout.addWidget(self.start_button)
        
        self.complete_button = QPushButton("Complete Mapping")
        self.complete_button.clicked.connect(self.complete_mapping)
        button_layout.addWidget(self.complete_button)
        
        self.save_button = QPushButton("Save Map")
        self.save_button.clicked.connect(self.save_map)
        self.save_button.setEnabled(False)
        button_layout.addWidget(self.save_button)
        
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)
    
    def start_mapping(self):
        self.start_button.setEnabled(False)
        self.mapping_controller.start_exploration()
    
    def complete_mapping(self):
        self.mapping_controller.complete_mapping()
        # GUI: 완료 후 start 버튼을 재활성화 (필요 시)
        self.start_button.setEnabled(True)
    
    def save_map(self):
        self.mapping_controller.save_map()
    
    def update_status(self):
        status = self.mapping_controller.mapping_status
        self.status_label.setText("Mapping status: " + status)
        if "completed" in status.lower():
            self.save_button.setEnabled(True)
        else:
            self.save_button.setEnabled(False)

def rclpy_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    exploration_node = FrontierExplorationNode()
    spin_thread = threading.Thread(target=rclpy_spin, args=(exploration_node,), daemon=True)
    spin_thread.start()
    app = QApplication(sys.argv)
    gui = MappingGUI(exploration_node)
    gui.show()
    app.exec_()
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
