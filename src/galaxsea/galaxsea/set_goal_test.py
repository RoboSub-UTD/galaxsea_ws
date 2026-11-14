import math
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def main():
    rclpy.init()
    node = Node('goal_sender')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    print("Waiting for navigate_to_pose action server...")
    client.wait_for_server()

    print("\nNav2 Goal Set")
    print("Format: x y angle_degrees")
    print("Commands: quit\n")

    while rclpy.ok():
        try:
            raw = input("> ").strip()
        except EOFError:
            break

        if not raw:
            continue

        if raw.lower() == 'quit':
            break

        parts = raw.split()
        if len(parts) != 3:
            print("  Error: expected 'x y angle_degrees'")
            continue

        try:
            x, y, deg = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  Error: x, y, and angle must be numbers")
            continue

        if not all(math.isfinite(value) for value in (x, y, deg)):
            print("  Error: x, y, and angle must be finite")
            continue

        yaw = math.radians(deg)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # A fixed map goal does not need a historical transform. Zero means
        # latest available TF, including when Nav2 replans later in the trip.
        # This also avoids using a stale /clock value after blocking on input().
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        print(f"  Sending goal: x={x}, y={y}, angle={deg}°")
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        if not rclpy.ok():
            break
        handle = send_future.result()
        if not handle.accepted:
            print("  Goal rejected")
            continue

        print("  Goal accepted; waiting for navigation to finish...")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        if not rclpy.ok():
            break
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("  Goal reached")
        else:
            print(f"  Navigation ended with status {status}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
