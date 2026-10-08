import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.signals import SignalHandlerOptions
# ============================================================
# IMPORTANT PARAMETERS / SEARCH INDEX
# ============================================================

# [SAFE_DISTANCE]
# safe_distance = 1.0
# 功能：前方障礙物小於此距離時，進入避障

# [TARGET_DISTANCE]
# target_distance = 0.8
# 功能：Side Following 時，希望與障礙物保持的側向距離

# [KP_TURNING]
# Kp_turning = 0.8
# 功能：控制轉向修正強度
# 使用位置：SIDE P、REJOINING yaw control

# [MAX_ANGULAR]
# max_angular = 0.5
# 功能：限制最大角速度，避免轉向太激烈

# [KP_FRONT]
# Kp_front = 1.0
# 功能：前方避障時，依照 front_error 增加轉向量

# [MIN_TURN]
# min_turn = 0.2
# 功能：確保前方避障時至少有基本轉向量

# [PASS_DISTANCE]
# pass_distance = 1.5
# 功能：側邊 LiDAR 超過此距離時，視為障礙物尾端已通過

# [SIDE_FOLLOW_DISTANCE]
# distance_traveled >= 1.0
# 功能：沿障礙物側邊走滿此距離後，從 AVOIDING 進入 PASSING

# [CLEARANCE_FORMULA]
# clearance_distance = 0.2 + 0.1 * obstacle_length
# 限制範圍：0.2 ~ 0.6 m
# 功能：障礙物越長，通過後保留越大的安全距離

# [REJOIN_ANGLE]
# rejoin_angle = 30 degrees
# 功能：REJOINING 時，以此斜角切回原本路線

# [Y_TOLERANCE]
# abs(y_error) <= 0.1
# 功能：距離原本路線小於 0.1 m 時，進入 RETURNING

# [YAW_TOLERANCE]
# abs(yaw_error) <= 5 degrees
# 功能：車頭與 original_yaw 相差小於 5 度時，回到 FORWARD

# ============================================================
# STATE MACHINE
# ============================================================

# FORWARD
#   -> AVOIDING
#   -> PASSING
#   -> REJOINING
#   -> RETURNING
#   -> FORWARD

# ============================================================
class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("ObstacleAvoidance")

        self.original_yaw = None
        self.avoid_direction = None
        self.current_yaw = 0.0
        self.current_y = 0.0
        self.original_y = None

        self.current_x = 0.0
        self.clearance_start_x = None
        self.state = "FORWARD"
        self.previous_error = 0.0
        self.target_distance = 0.8
        self.Kp_turning = 0.8
        self.max_angular = 0.5
        self.side_seen = False
        self.Kp_front = 1.0
        self.min_turn = 0.2 
        self.clearance_distance = 0.4
        self.side_follow_start_x = None
        self.side_follow_start_y = None
        self.obstacle_length = 0.0
        self.publisher =self.create_publisher(
            Twist,
            '/cmd_vel',
            10

        )
        self.scan_subscription =self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10

        )
        self.odom_subscription=self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )
        print("NODE INITIALIZED")


    def scan_callback(self, msg):
        front_ranges = []
        left_ranges = []
        right_ranges = []
        left_min_angle = math.radians(30)
        right_max_angle = math.radians(-30)
        left_max_angle = math.radians(90)
        right_min_angle = math.radians(-90)
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if math.isfinite(distance) and distance >= msg.range_min:
                if left_min_angle <= angle <= left_max_angle:
                    left_ranges.append(distance)
                elif right_min_angle <= angle <= right_max_angle:
                    right_ranges.append(distance)
                elif right_max_angle <= angle <= left_min_angle:
                    front_ranges.append(distance)
        front_distance = min(front_ranges) if front_ranges else float('inf')
        left_distance = min(left_ranges) if left_ranges else float('inf')
        right_distance = min(right_ranges) if right_ranges else float('inf') 
        self.get_logger().info(
            f"Left: {left_distance:.2f} | Front: {front_distance:.2f} | Right: {right_distance:.2f}"
        )
        print(f"STATE: {self.state}")

        cmd = Twist()
        safe_distance = 1.0# 前方障礙物小於此距離 → 開始避障 (m)
    

# 第一次看到障礙物：記住原本方向
        if front_distance < safe_distance and self.state == "FORWARD":
            self.original_yaw = self.current_yaw
            self.original_y = self.current_y
            self.state = "AVOIDING"
            self.side_seen = False
            self.side_follow_start_x = None

            print(
                f"Start avoiding, original yaw: "
                f"{math.degrees(self.original_yaw):.1f} degrees"
            )

# 1. 正在回到原本路線
        # 1. 正在回到原本路線
        if self.state == "REJOINING":
            rejoin_angle = math.radians(30)
            y_error = self.current_y - self.original_y
            
            print(
                f"Rejoining | Y error: {y_error:.2f} m | "
                f"Yaw: {math.degrees(self.current_yaw):.1f} deg"
            )
            
    # 車在原路的 -Y 側 → 往 +Y 靠
            if abs(y_error) > 0.1:
                if y_error>0:
                    target_yaw = self.original_yaw - rejoin_angle
                else:
                    target_yaw = self.original_yaw + rejoin_angle
                yaw_error = math.atan2(
                    math.sin(target_yaw - self.current_yaw),
                    math.cos(target_yaw - self.current_yaw)
                )
                angular_command = self.Kp_turning * yaw_error

                cmd.linear.x = 0.15
                cmd.angular.z = max(
                    -self.max_angular,
                    min(self.max_angular, angular_command)
                )
                print(
                    f"TARGET YAW: {math.degrees(target_yaw):.1f} | "
                    f"YAW ERROR: {math.degrees(yaw_error):.1f}"
                )

    # 已經回到原路附近
            else:
                self.state = "RETURNING"

                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

                print("Path rejoined, start returning yaw")
        elif self.state == "PASSING":

            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

            pass_distance = 1.5

    # 往右避障 → 障礙物在左側
            if self.avoid_direction == "right":

        # 第一次確認左側已經清空
                if left_distance >= pass_distance and self.clearance_start_x is None:
                    self.obstacle_length = abs(
                        self.current_x - self.side_follow_start_x
                        )
                    self.clearance_start_x = self.current_x
                    self.clearance_distance = 0.2 + 0.1 * self.obstacle_length
                    self.clearance_distance = max(
                        0.2,min(0.6, self.clearance_distance)
                    )
                    print(
                        f"Obstacle length: {self.obstacle_length:.2f} m | "
                        f"Clearance distance: {self.clearance_distance:.2f} m"
                    )
                    print("Side clear, start extra clearance")

    # 往左避障 → 障礙物在右側
            elif self.avoid_direction == "left":

        # 第一次確認右側已經清空
                if right_distance >= pass_distance and self.clearance_start_x is None:
                    self.obstacle_length = abs(
                        self.current_x - self.side_follow_start_x
                    )
                    self.clearance_start_x = self.current_x
                    self.clearance_distance = 0.2 + 0.1 * self.obstacle_length
                    self.clearance_distance = max(0.2,
                            min(0.6, self.clearance_distance)
                        )
                    print(
                        f"Obstacle length: {self.obstacle_length:.2f} m | "
                        f"Clearance distance: {self.clearance_distance:.2f} m"
                    )
                    print("Side clear, start extra clearance")

    # 已經開始計算額外前進距離
            if self.clearance_start_x is not None:

                distance_traveled = abs(
                    self.current_x - self.clearance_start_x
                )

                print(
                    f"Extra clearance: {distance_traveled:.2f} m"
                )

        # 多走 0.8 m 之後才開始回原路
                if distance_traveled >= self.clearance_distance:

                    self.state = "REJOINING"
                    self.clearance_start_x = None

                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                    print("Enough clearance, start rejoining")
        elif self.state =="RETURNING":
            yaw_error = math.atan2(
            math.sin(self.current_yaw - self.original_yaw),
            math.cos(self.current_yaw - self.original_yaw)
            )
            print(
            f"Aligning | Yaw error: "
            f"{math.degrees(yaw_error):.1f} deg"
            )
            if  yaw_error > math.radians(5):
                cmd.linear.x = 0.0
                cmd.angular.z = -0.5

    # 偏右 → 左轉
            elif yaw_error < math.radians(-5):
                
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5

    # 已經接近原方向
            else:
                cmd.linear.x = 0.3
                cmd.angular.z = 0.0

                self.state = "FORWARD"

                self.original_yaw = None
                self.original_y = None
                self.avoid_direction = None
                self.clearance_start_x = None
                self.side_seen = False
                self.side_follow_start_x = None
                self.obstacle_length = 0.0
                self.clearance_distance = 0.4
                print("Returned to original direction")

# 3. 正常直走
        elif front_distance >= safe_distance and self.state == "FORWARD":

            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

# 4. 左右都被擋住
        elif left_distance < safe_distance and right_distance < safe_distance:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

# 5. 左邊比較空 → 左轉
        elif self.state == "AVOIDING":
            # 第一次決定方向
            if self.avoid_direction is None:

                if left_distance > right_distance:
                    self.avoid_direction = "left"

                elif right_distance > left_distance:
                    self.avoid_direction = "right"

                else:
                    self.avoid_direction = "right"

    # =========================
    # 往右繞
    # =========================
            elif self.avoid_direction == "right":

        # 前面還有障礙物：
        # 先優先右轉，把車頭轉開
                if front_distance < safe_distance:

                    front_error = safe_distance - front_distance

                    angular_command = -(
                        self.min_turn
                        + self.Kp_front * front_error
                    )

                    cmd.linear.x = 0.05
                    cmd.angular.z = max(
                        -self.max_angular,
                        min(self.max_angular, angular_command)
                    )

                    print(
                        f"FRONT AVOID | error={front_error:.2f} | "
                        f"angular={cmd.angular.z:.2f}"
                    )

        # 前面已經清掉，而且左邊看得到障礙物：
        # 開始側向 P control
                elif math.isfinite(left_distance):

                    self.side_seen = True
                    if self.side_follow_start_x is None:
                        self.side_follow_start_x = self.current_x
                        self.side_follow_start_y = self.current_y

                    error = left_distance - self.target_distance
                    distance_traveled = abs(
                    self.current_x - self.side_follow_start_x
                    )

                    print(
                        f"SIDE FOLLOW DISTANCE: {distance_traveled:.2f} m"
                    )
                    angular_command = self.Kp_turning * error

                    cmd.linear.x = 0.1
                    cmd.angular.z = max(
                        -self.max_angular,
                        min(self.max_angular, angular_command)
                    )

                    print(
                        f"SIDE P | error={error:.2f} | "
                        f"angular={cmd.angular.z:.2f}"
                    )
                    if distance_traveled >= 1.0:
                        self.state = "PASSING"
                        self.clearance_start_x = None

                        cmd.linear.x = 0.2
                        cmd.angular.z = 0.0

                        print("Obstacle passed → ENTERING PASSING")
                    

     
    # 往左繞
    # =========================
            elif self.avoid_direction == "left":

                if front_distance < safe_distance:

                    front_error = safe_distance - front_distance

                    angular_command = (
                        self.min_turn
                        + self.Kp_front * front_error
                    )

                    cmd.linear.x = 0.05
                    cmd.angular.z = max(
                        -self.max_angular,
                        min(self.max_angular, angular_command)
                    )

                elif math.isfinite(right_distance):

                    self.side_seen = True
                    if self.side_follow_start_x is None:
                        self.side_follow_start_x = self.current_x

                    error = right_distance - self.target_distance
                    distance_traveled = abs(
                    self.current_x - self.side_follow_start_x
                    )

                    print(
                        f"SIDE FOLLOW DISTANCE: {distance_traveled:.2f} m"
                    )
                    angular_command = -self.Kp_turning * error

                    cmd.linear.x = 0.1
                    cmd.angular.z = max(
                        -self.max_angular,
                        min(self.max_angular, angular_command)
                    )
                    if distance_traveled >= 1.0:
                        self.state = "PASSING"
                        self.clearance_start_x = None

                        cmd.linear.x = 0.2
                        cmd.angular.z = 0.0

                        print("Obstacle passed → ENTERING PASSING")

            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        self.publisher.publish(cmd)
    def stop_robot(self):
        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.publisher.publish(cmd)

        self.get_logger().info("Stop command published")

    def odom_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = yaw
        self.current_y = msg.pose.pose.position.y
        self.current_x = msg.pose.pose.position.x

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Ctrl+C received, stopping robot...")

        node.stop_robot()

        rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()