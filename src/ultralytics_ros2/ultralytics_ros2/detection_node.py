import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from ultralytics import YOLO
import cv2
import numpy as np
import torch
import math

class YOLOPersonTracker(Node):
    def __init__(self):
        super().__init__('yolo_person_tracker')
        
        # 参数声明
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model', 'yolov8n.pt'),
                ('input_image_topic', '/camera/image_raw'),
                ('enable_cuda', True),
                ('conf_threshold', 0.5),
                ('linear_speed', 0.25),  # 固定线速度
                ('angular_speed', 0.3),  # 固定角速度
                ('mode', 1),  # 1:sleep 2:follow
                ('dead_zone_x', 0.1),  # 水平死区
                ('dead_zone_area', 0.2),  # 面积死区设置为0.2
                ('stop_delay', 0.5),  # 停止延迟（秒），防止抖动
                ('target_area_ratio', 0.6),  # 固定的目标面积比例（占图像总面积的比例），增加到0.6
            ]
        )
        
        # 参数获取
        model_path = self.get_parameter('model').value
        input_topic = self.get_parameter('input_image_topic').value
        enable_cuda = self.get_parameter('enable_cuda').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        
        # 固定速度参数
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.mode = self.get_parameter('mode').value
        self.dead_zone_x = self.get_parameter('dead_zone_x').value
        self.dead_zone_area = self.get_parameter('dead_zone_area').value  # 0.2
        self.stop_delay = self.get_parameter('stop_delay').value
        self.target_area_ratio = self.get_parameter('target_area_ratio').value  # 固定的目标面积比例

        # 检查CUDA可用性
        self.cuda_available = enable_cuda and torch.cuda.is_available()
        if enable_cuda and not self.cuda_available:
            self.get_logger().warning("CUDA requested but not available. Using CPU instead.")

        # 初始化YOLO模型
        self.model = YOLO(model_path)
        if self.cuda_available:
            self.model.to('cuda')
            self.get_logger().info("Using CUDA for inference")
        else:
            self.get_logger().info("Using CPU for inference")
        
        self.model.fuse()

        # 图像处理工具
        self.bridge = CvBridge()
        
        # 订阅/发布
        self.sub = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.pub_image = self.create_publisher(Image, 'detected_image', 10)
        self.pub_detections = self.create_publisher(Detection2DArray, 'detections', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 模式切换订阅
        self.mode_sub = self.create_subscription(Int8, '/mode', self.mode_callback, 10)
        
        # 帧率计算
        self.frame_count = 0
        self.last_time = self.get_clock().now()

        # 只检测人的类别ID
        self.person_class_id = 0
        
        # 目标跟踪状态
        self.target_person = None
        self.image_center = None
        self.image_width = 0
        self.image_height = 0
        
        # 停止计时器
        self.stop_timer = None
        self.last_cmd_vel = Twist()
        
        # 打印参数
        self.get_logger().info(f"Fixed speeds - linear: {self.linear_speed}, angular: {self.angular_speed}")
        self.get_logger().info(f"Dead zones - x: {self.dead_zone_x}, area: {self.dead_zone_area}")
        self.get_logger().info(f"Stop delay: {self.stop_delay}")
        self.get_logger().info(f"Target area ratio: {self.target_area_ratio}")
        self.get_logger().info(f"Mode: {self.mode} (1:sleep, 2:follow)")

    def mode_callback(self, msg):
        """模式切换回调"""
        self.mode = msg.data
        if self.mode == 1:
            self.get_logger().info("切换到睡眠模式")
            # 发布停止命令
            stop_cmd = Twist()
            self.pub_cmd_vel.publish(stop_cmd)
            self.last_cmd_vel = stop_cmd
        elif self.mode == 2:
            self.get_logger().info("切换到跟随模式")

    def find_closest_to_center(self, boxes):
        """找到最靠近图像中心的人员"""
        if self.image_width == 0 or self.image_height == 0:
            return None
            
        center_x = self.image_width // 2
        min_distance = float('inf')
        closest_box = None
        
        for box in boxes:
            if int(box.cls) == self.person_class_id:
                box_center_x = float(box.xywh[0][0])
                
                # 计算到图像中心的水平距离
                distance = abs(box_center_x - center_x)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_box = box
        
        return closest_box

    def calculate_control_command(self, target_box):
        """基于固定速度计算机器人控制命令"""
        cmd_vel = Twist()
        
        if target_box is None or self.mode != 2:
            # 没有检测到目标或不在跟随模式，停止移动
            return cmd_vel, 0, 0, 0, "NO_TARGET"
        
        # 获取目标框信息
        box_center_x = float(target_box.xywh[0][0])
        box_width = float(target_box.xywh[0][2])
        box_height = float(target_box.xywh[0][3])
        
        # 计算当前框面积
        current_area = box_width * box_height
        image_area = self.image_width * self.image_height
        current_area_ratio = current_area / image_area
        
        # 计算与图像中心的偏差（归一化到[-1, 1]）
        center_x = self.image_width // 2
        x_error = (box_center_x - center_x) / center_x  # 水平偏差
        
        # 计算面积误差（相对于固定目标面积）
        area_error = (current_area_ratio - self.target_area_ratio) / self.target_area_ratio
        
        # 调试信息
        self.get_logger().debug(f"x_error: {x_error:.3f}, area_error: {area_error:.3f}")
        self.get_logger().debug(f"dead_zone_x: {self.dead_zone_x}, dead_zone_area: {self.dead_zone_area}")
        
        # 检查是否在死区内
        in_deadzone_x = abs(x_error) <= self.dead_zone_x
        in_deadzone_area = abs(area_error) <= self.dead_zone_area
        
        self.get_logger().debug(f"in_deadzone_x: {in_deadzone_x}, in_deadzone_area: {in_deadzone_area}")
        
        # 如果完全在死区内，停止所有运动
        if in_deadzone_x and in_deadzone_area:
            # 设置停止计时器
            if self.stop_timer is None:
                self.stop_timer = self.get_clock().now()
                return Twist(), x_error, 0, area_error, "IN_DEADZONE"
            else:
                # 检查是否已经过了停止延迟
                elapsed_time = (self.get_clock().now() - self.stop_timer).nanoseconds / 1e9
                if elapsed_time >= self.stop_delay:
                    return Twist(), x_error, 0, area_error, "STOPPED"
                else:
                    # 继续之前的运动，但准备停止
                    return self.last_cmd_vel, x_error, 0, area_error, "PREPARE_STOP"
        else:
            # 不在死区内，重置停止计时器
            self.stop_timer = None
        
        # 角度控制：如果目标在死区外，以固定角速度转向目标
        if not in_deadzone_x:
            # 目标在右侧，需要左转（负角速度）
            if x_error > 0:
                cmd_vel.angular.z = -self.angular_speed
                angular_status = "TURN_LEFT"
            # 目标在左侧，需要右转（正角速度）
            else:
                cmd_vel.angular.z = self.angular_speed
                angular_status = "TURN_RIGHT"
        else:
            cmd_vel.angular.z = 0.0
            angular_status = "NO_TURN"
        
        # 距离控制：如果目标框大小在死区外，以固定线速度前进或后退
        if not in_deadzone_area:
            # 目标框太小（人太远），需要前进
            if area_error < 0:
                cmd_vel.linear.x = self.linear_speed
                linear_status = "FORWARD"
            # 目标框太大（人太近），需要后退
            else:
                cmd_vel.linear.x = -self.linear_speed
                linear_status = "BACKWARD"
        else:
            cmd_vel.linear.x = 0.0
            linear_status = "NO_MOVE"
        
        # 保存当前命令
        self.last_cmd_vel = cmd_vel
        
        status = f"{linear_status}_{angular_status}" if linear_status != "NO_MOVE" or angular_status != "NO_TURN" else "IN_DEADZONE"
        
        return cmd_vel, x_error, 0, area_error, status

    def image_callback(self, msg):
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_height, self.image_width = cv_image.shape[:2]
            self.image_center = (self.image_width // 2, self.image_height // 2)
            
            # 执行推理 - 只检测人
            results = self.model.predict(
                source=cv_image,
                conf=self.conf_threshold,
                classes=[self.person_class_id],
                verbose=False,
                device='cuda' if self.cuda_available else 'cpu'
            )

            # 准备检测结果消息
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            # 找到最靠近中心的人员作为目标
            target_box = None
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                target_box = self.find_closest_to_center(results[0].boxes)
                self.target_person = target_box
            
            # 计算机器人控制命令
            cmd_vel, x_error, y_error, area_error, status = self.calculate_control_command(target_box)
            
            # 发布控制命令
            self.pub_cmd_vel.publish(cmd_vel)
            
            # 绘制检测结果
            annotated_image = results[0].plot() if results[0].boxes is not None else cv_image.copy()
            
            # 在图像上绘制中心十字线和目标信息
            cv2.line(annotated_image, 
                    (self.image_center[0] - 20, self.image_center[1]), 
                    (self.image_center[0] + 20, self.image_center[1]), 
                    (0, 255, 0), 2)
            cv2.line(annotated_image, 
                    (self.image_center[0], self.image_center[1] - 20), 
                    (self.image_center[0], self.image_center[1] + 20), 
                    (0, 255, 0), 2)
            
            # 绘制死区范围
            dead_zone_left = int(self.image_center[0] - self.dead_zone_x * self.image_width)
            dead_zone_right = int(self.image_center[0] + self.dead_zone_x * self.image_width)
            cv2.line(annotated_image, 
                    (dead_zone_left, 0), 
                    (dead_zone_left, self.image_height), 
                    (255, 255, 0), 1)
            cv2.line(annotated_image, 
                    (dead_zone_right, 0), 
                    (dead_zone_right, self.image_height), 
                    (255, 255, 0), 1)
            
            # 如果找到目标，绘制目标框和连线
            if target_box is not None:
                box_center_x = int(float(target_box.xywh[0][0]))
                box_center_y = int(float(target_box.xywh[0][1]))
                box_width = int(float(target_box.xywh[0][2]))
                box_height = int(float(target_box.xywh[0][3]))
                
                # 计算当前框面积
                current_area = box_width * box_height
                image_area = self.image_width * self.image_height
                current_area_ratio = current_area / image_area
                
                # 绘制目标框
                cv2.rectangle(annotated_image,
                            (box_center_x - box_width // 2, box_center_y - box_height // 2),
                            (box_center_x + box_width // 2, box_center_y + box_height // 2),
                            (0, 0, 255), 3)
                
                # 绘制到中心的连线
                cv2.line(annotated_image, 
                        (box_center_x, box_center_y), 
                        self.image_center, 
                        (0, 255, 255), 2)
                
                # 绘制目标中心点
                cv2.circle(annotated_image, (box_center_x, box_center_y), 5, (0, 0, 255), -1)
                
                # 在目标框上方显示状态
                cv2.putText(annotated_image, status, 
                           (box_center_x - box_width // 2, box_center_y - box_height // 2 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # 显示面积信息
                area_info = f"Area: {current_area_ratio:.3f} (Target: {self.target_area_ratio:.3f})"
                
                cv2.putText(annotated_image, area_info,
                           (box_center_x - box_width // 2, box_center_y - box_height // 2 - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # 计算并显示帧率
            current_time = self.get_clock().now()
            delta_time = current_time - self.last_time
            fps = 1e9 / delta_time.nanoseconds if delta_time.nanoseconds > 0 else 0.0
            
            # 在图像上显示信息
            person_count = len(results[0].boxes) if results[0].boxes is not None else 0
            mode_text = "SLEEP" if self.mode == 1 else "FOLLOW"
            info_text = [
                f'FPS: {fps:.2f}',
                f'Persons: {person_count}',
                f'Mode: {mode_text}',
                f'Target: {"Found" if target_box else "None"}',
                f'X Error: {x_error:.3f}' if target_box else '',
                f'Area Error: {area_error:.3f}' if target_box else '',
                f'Status: {status}',
                f'Linear X: {cmd_vel.linear.x:.3f}',
                f'Angular Z: {cmd_vel.angular.z:.3f}',
                f'Dead Zone X: ±{self.dead_zone_x}',
                f'Dead Zone Area: ±{self.dead_zone_area}',
                f'Stop Delay: {self.stop_delay}s',
                f'Target Area: {self.target_area_ratio:.3f}'
            ]
            
            for i, text in enumerate(info_text):
                if text:  # 只显示非空文本
                    cv2.putText(annotated_image, text, (10, 30 + i * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # 发布带检测结果的图像
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8'))
            
            # 填充检测结果消息
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    if int(box.cls) == self.person_class_id:
                        detection = Detection2D()
                        detection.bbox = BoundingBox2D()
                        
                        detection.bbox.center.position.x = float(box.xywh[0][0])
                        detection.bbox.center.position.y = float(box.xywh[0][1])
                        detection.bbox.size_x = float(box.xywh[0][2])
                        detection.bbox.size_y = float(box.xywh[0][3])
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = "person"
                        hypothesis.hypothesis.score = float(box.conf)
                        detection.results.append(hypothesis)
                        
                        # 标记是否为跟踪目标
                        if box == target_box:
                            detection.id = "target"
                        
                        detections_msg.detections.append(detection)
            
            # 发布检测结果
            self.pub_detections.publish(detections_msg)
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    tracker = YOLOPersonTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        # 停止机器人
        stop_cmd = Twist()
        tracker.pub_cmd_vel.publish(stop_cmd)
        tracker.get_logger().info("Stopping robot and shutting down...")
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()