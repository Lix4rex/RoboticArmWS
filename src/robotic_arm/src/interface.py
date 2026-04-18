#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import TargetPosition

import threading
import tkinter as tk
import math


class CircleGUI(Node):

    def __init__(self):
        super().__init__('circle_gui_publisher')

        # ROS publisher
        self.publisher_ = self.create_publisher(
            TargetPosition,
            '/target_position',
            10
        )

        self.timer = self.create_timer(0.02, self.publish_loop)  # 50 Hz

        self.root = tk.Tk()
        self.root.title("Target Position GUI")

        self.width = 500
        self.height = 300
        self.radius = 120

        self.canvas = tk.Canvas(self.root, width=self.width, height=self.height, bg="white")
        self.canvas.pack()

        self.cx = self.width // 2
        self.cy = self.height - 50

        self.x = self.cx
        self.y = self.cy - self.radius

        self.dragging = False

        self.draw_scene()

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        self.get_logger().info("Tkinter GUI started")

        self.last_msg = TargetPosition()

    def draw_scene(self):
        self.canvas.delete("all")

        self.canvas.create_arc(
            self.cx - self.radius,
            self.cy - self.radius,
            self.cx + self.radius,
            self.cy + self.radius,
            start=0,
            extent=180,
            outline="black",
            width=2
        )

        self.canvas.create_oval(
            self.x - 8, self.y - 8,
            self.x + 8, self.y + 8,
            fill="red"
        )

    def constrain_to_semicircle(self, x, y):
        dx = x - self.cx
        dy = y - self.cy

        dist = math.sqrt(dx**2 + dy**2)

        if dist > self.radius:
            scale = self.radius / dist
            dx *= scale
            dy *= scale

        if dy > 0:
            dy = 0

        return self.cx + dx, self.cy + dy

    def on_click(self, event):
        self.dragging = True

    def on_drag(self, event):
        if not self.dragging:
            return

        x, y = self.constrain_to_semicircle(event.x, event.y)
        self.x = x
        self.y = y

        self.draw_scene()

        self.last_msg.x_target = -10*float((self.x - self.cx) / self.radius)
        self.last_msg.y_target = 0.0
        self.last_msg.z_target = 10*float((self.cy - self.y) / self.radius)

    def on_release(self, event):
        self.dragging = False

    def publish_loop(self):
        self.publisher_.publish(self.last_msg)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)

    node = CircleGUI()

    # ROS spin dans un thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        node.run()  # Tkinter mainloop
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()