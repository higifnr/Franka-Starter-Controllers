#!/usr/bin/env python3

import rospy
import tkinter as tk
from std_msgs.msg import Float64MultiArray

class JointVelTeleopGUI:
    def __init__(self):
        rospy.init_node("joint_velocity_teleop_gui")
        rospy.on_shutdown(self.shutdown)

        self.cmd_pub_topic = "/simple_joint_velocity_controller/joint_velocity_commands"
        #self.cmd_pub_topic = "/unified_velocity_controller/joint_velocity_commands"
        self.cmd_pub = rospy.Publisher(self.cmd_pub_topic, Float64MultiArray, queue_size=1)

        self.root = tk.Tk()
        self.root.title("Joint Velocity Teleop")

        # Current command (7 joint velocities)
        self.cmd = [0.0] * 7
        self.active_cmd = None

        # Scaling factor for velocity magnitude
        self.scale_val = tk.DoubleVar(value=0.5)

        self.error_var = tk.StringVar()
        self.error_var.set("Error: N/A")

        self.make_gui()

        # Example error subscriber (optional)
        rospy.Subscriber("/error", Float64MultiArray, self.error_callback)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.loop()

    def shutdown(self):
        try:
            self.root.quit()
            self.root.destroy()
        except Exception as e:
            rospy.logdebug("safe_stop_gui exception: %s", e)

    def make_gui(self):
        # Create buttons for each joint +/- velocity control
        # Layout: 7 joints, each has a "-" and "+" button

        for i in range(7):
            joint_label = tk.Label(self.root, text=f"Joint {i+1}")
            joint_label.grid(row=i, column=0, padx=5, pady=5)

            self.make_button(f"-", lambda j=i: self.set_cmd(j, -1), i, 1)
            self.make_button(f"+", lambda j=i: self.set_cmd(j, 1), i, 2)

        # Velocity scale slider
        tk.Label(self.root, text="Velocity Scale").grid(row=7, column=0, pady=(10, 0))
        scale_slider = tk.Scale(self.root, from_=0.0, to=1.0,
                                resolution=0.01, orient=tk.HORIZONTAL,
                                variable=self.scale_val)
        scale_slider.grid(row=7, column=1, columnspan=2, sticky="we")

        # Error display at bottom
        error_label = tk.Label(self.root, textvariable=self.error_var,
                               fg="red", font=("Arial", 10))
        error_label.grid(row=8, column=0, columnspan=3, pady=(5, 10))

    def make_button(self, text, cmd_fn, row, col):
        btn = tk.Button(self.root, text=text, width=4, height=2)
        btn.grid(row=row, column=col, padx=5, pady=5)
        btn.bind("<ButtonPress>", lambda e: self.start_motion(cmd_fn))
        btn.bind("<ButtonRelease>", lambda e: self.stop_motion())
        return btn

    def set_cmd(self, joint_index, direction):
        scale = self.scale_val.get()
        # Reset command to zero first
        self.cmd = [0.0] * 7
        self.cmd[joint_index] = direction * scale * 0.1  # 0.1 rad/s base velocity scaled

    def start_motion(self, cmd_fn):
        cmd_fn()
        self.active_cmd = Float64MultiArray(data=self.cmd)

    def stop_motion(self):
        self.cmd = [0.0] * 7
        self.cmd_pub.publish(Float64MultiArray(data=self.cmd))
        self.active_cmd = None

    def error_callback(self, msg):
        # Just display any error message or data you want
        # For demo, assume msg.data is a list and we show first value
        if msg.data:
            self.error_var.set(f"Error: {msg.data[0]:.6f}")
        else:
            self.error_var.set("Error: N/A")

    def loop(self):
        if self.active_cmd and not rospy.is_shutdown():
            self.cmd_pub.publish(self.active_cmd)
        self.root.after(10, self.loop)

    def on_close(self):
        self.stop_motion()
        self.root.destroy()


if __name__ == "__main__":
    JointVelTeleopGUI().root.mainloop()
