#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import time

class SetGoalNode(Node):
    def __init__(self):
        super().__init__('set_goal_node')
        
        # 파라미터 선언
        self.declare_parameter('goal_x', 0.8)
        self.declare_parameter('goal_y', -5.0)
        self.declare_parameter('goal_yaw', 0.0)
        
        # 파라미터 가져오기
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        
        # Nav2 액션 클라이언트 생성
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 목표 지점 설정 타이머 (5초 후 실행)
        self.timer = self.create_timer(5.0, self.send_goal)
        self.get_logger().info('목표 지점 설정 노드가 시작되었습니다. 5초 후 목표 지점으로 이동을 시작합니다.')
    
    def send_goal(self):
        # 타이머 취소 (한 번만 실행)
        self.timer.cancel()
        
        # Nav2 액션 서버가 활성화될 때까지 대기
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 액션 서버를 기다리는 중...')
        
        # 목표 지점 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 목표 위치 설정
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        
        # 목표 방향 설정 (쿼터니언)
        # 간단한 예제이므로 yaw만 사용 (roll, pitch는 0)
        import math
        cy = math.cos(self.goal_yaw * 0.5)
        sy = math.sin(self.goal_yaw * 0.5)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sy
        goal_msg.pose.pose.orientation.w = cy
        
        self.get_logger().info(f'목표 지점으로 이동 중: x={self.goal_x}, y={self.goal_y}, yaw={self.goal_yaw}')
        
        # 목표 지점으로 이동 요청 전송
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # 목표 요청 결과 콜백 설정
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('목표 지점이 거부되었습니다.')
            return
        
        self.get_logger().info('목표 지점이 수락되었습니다.')
        
        # 목표 실행 결과 콜백 설정
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 지점에 도착했습니다!')
        else:
            self.get_logger().error(f'목표 지점 도달 실패: {status}')
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 필요한 경우 피드백 처리
        # self.get_logger().info(f'현재 위치: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = SetGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 