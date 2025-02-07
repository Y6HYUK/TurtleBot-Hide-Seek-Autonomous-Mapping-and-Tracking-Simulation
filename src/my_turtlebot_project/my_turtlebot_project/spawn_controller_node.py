#!/usr/bin/env python3
import sys
import threading
import subprocess
import os
import random
import time

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# SpawnControllerGUI는 ROS 노드와 QWidget을 동시에 상속받아 GUI와 spawn/autonomous 제어를 수행합니다.
class SpawnControllerGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'spawn_controller_node')
        QWidget.__init__(self)
        self.setWindowTitle("Spawn Controller GUI")
        self.setMinimumSize(500, 300)
        self.setStyleSheet("""
            QWidget {
                background-color: #f0f0f0;
                font-family: Arial, sans-serif;
                font-size: 14px;
            }
            QPushButton {
                background-color: #007ACC;
                color: white;
                border: none;
                padding: 10px 16px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #005A9E;
            }
            QLabel {
                color: #333333;
            }
        """)

        # 상태 라벨
        self.status_label = QLabel("Status: Ready", self)
        
        # Spawn 버튼
        self.spawn_button = QPushButton("Spawn Additional Robot", self)
        self.spawn_button.clicked.connect(self.spawn_robot)
        
        # Autonomous Movement 시작/중지 버튼
        self.start_auto_button = QPushButton("Start Autonomous Movement", self)
        self.start_auto_button.clicked.connect(self.start_autonomous_movement)
        self.start_auto_button.setEnabled(False)
        self.stop_auto_button = QPushButton("Stop Autonomous Movement", self)
        self.stop_auto_button.clicked.connect(self.stop_autonomous_movement)
        self.stop_auto_button.setEnabled(False)

        # 레이아웃 구성
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.spawn_button)
        button_layout.addWidget(self.start_auto_button)
        button_layout.addWidget(self.stop_auto_button)
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.status_label)
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)
        
        # 내부 변수 초기화
        self.robot_counter = 2           # 추가 로봇은 2번부터 시작
        self.last_spawned_namespace = None  # 마지막에 소환한 로봇의 네임스페이스 (예: "turtlebot3_2")
        self.auto_active = False         # 자율 주행 모드 활성화 여부
        self.auto_thread = None          # 자율 주행 쓰레드
        
        # GUI 업데이트 타이머
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)

    def update_status(self):
        # 상태 라벨 업데이트 (필요에 따라 추가 정보 표시)
        pass

    def spawn_robot(self):
        self.status_label.setText("Spawning additional robot...")
        threading.Thread(target=self._spawn_robot_command, daemon=True).start()

    def _spawn_robot_command(self):
        entity_name = f"turtlebot3_{self.robot_counter}"
        spawn_x = 0.95 * self.robot_counter  # spawn 좌표 (필요에 따라 조정)
        spawn_y = 0.0
        # Waffle Pi 모델을 사용하도록 모델 파일 경로 수정
        cmd = (
            f"ros2 run gazebo_ros spawn_entity.py "
            f"-file $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf "
            f"-entity {entity_name} "
            f"-robot_namespace /{entity_name} "
            f"-x {spawn_x} -y {spawn_y} -z 0.01 -Y 0.0 -unpause"
        )
        try:
            subprocess.run(cmd, shell=True, check=True)
            self.status_label.setText(f"Successfully spawned {entity_name}!")
            self.last_spawned_namespace = entity_name
            self.robot_counter += 1
            self.start_auto_button.setEnabled(True)
        except subprocess.CalledProcessError as e:
            self.status_label.setText("Spawn additional robot failed: " + str(e))

    def start_autonomous_movement(self):
        if not self.last_spawned_namespace:
            self.status_label.setText("No spawned robot to control!")
            return
        self.status_label.setText(f"Starting autonomous movement for {self.last_spawned_namespace}...")
        self.auto_active = True
        self.start_auto_button.setEnabled(False)
        self.stop_auto_button.setEnabled(True)
        # 시작 시, cmd_vel 토픽에 대한 퍼블리셔 생성 (네임스페이스 적용)
        self.auto_pub = self.create_publisher(Twist, f"/{self.last_spawned_namespace}/cmd_vel", 10)
        # 자율 주행 쓰레드 시작
        self.auto_thread = threading.Thread(target=self._autonomous_movement_loop, daemon=True)
        self.auto_thread.start()

    def _autonomous_movement_loop(self):
        # 단순 랜덤 워크 방식: 일정 주기로 random twist 메시지 발행
        rate = 2  # 초당 2번 명령 발행 (0.5초 주기)
        while self.auto_active:
            twist = Twist()
            # 간단하게 전진, 약간의 회전 명령을 랜덤 생성
            twist.linear.x = random.uniform(0.1, 0.3)
            twist.angular.z = random.uniform(-0.5, 0.5)
            self.auto_pub.publish(twist)
            time.sleep(1.0 / rate)
        # auto_active가 False가 되면, 로봇을 정지시키는 명령 한 번 발행
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.auto_pub.publish(twist)
        self.status_label.setText(f"Autonomous movement stopped for {self.last_spawned_namespace}.")

    def stop_autonomous_movement(self):
        self.auto_active = False
        self.start_auto_button.setEnabled(True)
        self.stop_auto_button.setEnabled(False)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    spawn_gui = SpawnControllerGUI()
    spawn_gui.show()
    # rclpy.spin()는 별도의 스레드에서 실행
    threading.Thread(target=rclpy.spin, args=(spawn_gui,), daemon=True).start()
    app.exec_()
    spawn_gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
