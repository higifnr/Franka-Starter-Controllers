#!/usr/bin/env python3

# GUI node with all possible cartesian movements

import rospy
import tkinter as tk
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

class EEVelTeleopGUI:
    """A GUI including cartesian movement commands"""
    def __init__(self):
        rospy.init_node("cartesian_velocity_teleop_gui")
        rospy.on_shutdown(self.shutdown)
        self.open_loop = rospy.get_param("~open_loop", False)
        self.arm_id         = rospy.get_param("/arm_id", "panda")
        self.cmd_pub_topic =  "/ee_velocity_cmd"
        self.cmd_pub_topic =  "/unified_velocity_controller/cartesian_velocity_commands"
        self.work_frame     = f"{self.arm_id}_EE"
        if self.open_loop:
            self.cmd_pub_topic = "/cartesian_velocity_controller/cartesian_velocity_command"
        self.cmd_pub = rospy.Publisher(self.cmd_pub_topic, Twist, queue_size=1)
        self.frame_pub = rospy.Publisher("/work_frame", String, queue_size=1)
        self.filter_pub = rospy.Publisher("/unified_velocity_controller/joint_velocity_filter", Float64, queue_size=1)

        self.root = tk.Tk()
        self.root.title("Cartesian Space Velocity Teleop")
        self.active_cmd = None
        self.cmd = Twist()

        # default scaling coefficients
        self.lin_scale_val = tk.DoubleVar(value=1.0)
        self.ang_scale_val = tk.DoubleVar(value=1.0)
        self.old_filter_val = 0.1
        self.filter_val = tk.DoubleVar(value=self.old_filter_val)
        

        self.error_var = tk.StringVar()
        self.error_var.set("Error: N/A")

        self.make_gui()

        # Subscribers
        rospy.Subscriber("/rcm_to_axis_error", Float64, self.error_callback)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.loop()

    def shutdown(self,):
        try:
            self.root.quit()     # stops mainloop
            self.root.destroy()  # destroys widgets & window
        except Exception as e:
            # ignore if the window is already closed
            rospy.logdebug("safe_stop_gui exception: %s", e)

    def make_gui(self):
        # Movement buttons
        self.make_button("← X-", lambda: self.set_cmd(linear=(-1, 0, 0)), 1, 0)
        self.make_button("→ X+", lambda: self.set_cmd(linear=(1, 0, 0)), 1, 2)
        self.make_button("↓ Y-", lambda: self.set_cmd(linear=(0, -1, 0)), 2, 1)
        self.make_button("↑ Y+", lambda: self.set_cmd(linear=(0, 1, 0)), 0, 1)
        self.make_button("⇣ Z-", lambda: self.set_cmd(linear=(0, 0, -1)), 3, 0)
        self.make_button("⇡ Z+", lambda: self.set_cmd(linear=(0, 0, 1)), 3, 2)

        # Rotation buttons
        self.make_button("⟲ Rx-", lambda: self.set_cmd(angular=(-1, 0, 0)), 4, 0)
        self.make_button("⟳ Rx+", lambda: self.set_cmd(angular=(1, 0, 0)), 4, 2)
        self.make_button("⟱ Ry-", lambda: self.set_cmd(angular=(0, -1, 0)), 5, 0)
        self.make_button("⟰ Ry+", lambda: self.set_cmd(angular=(0, 1, 0)), 5, 2)
        self.make_button("⤵ Rz-", lambda: self.set_cmd(angular=(0, 0, -1)), 6, 0)
        self.make_button("⤴ Rz+", lambda: self.set_cmd(angular=(0, 0, 1)), 6, 2)


        # Linear speed slider
        tk.Label(self.root, text="Linear Scale").grid(row=8, column=0, pady=(10,0))
        lin_slider = tk.Scale(self.root, from_=0.0, to=2.0, 
                              resolution=0.01, orient=tk.HORIZONTAL,
                              variable=self.lin_scale_val)
        lin_slider.grid(row=8, column=1, columnspan=2, sticky="we")

        # Angular speed slider
        tk.Label(self.root, text="Angular Scale").grid(row=9, column=0)
        ang_slider = tk.Scale(self.root, from_=0.0, to=2.0, 
                              resolution=0.01, orient=tk.HORIZONTAL,
                              variable=self.ang_scale_val)
        ang_slider.grid(row=9, column=1, columnspan=2, sticky="we")

        # low pass filter parameter slider
        tk.Label(self.root, text="Filter parameter").grid(row=10, column=0)
        ang_slider = tk.Scale(self.root, from_=0.0, to=1.0, 
                              resolution=0.01, orient=tk.HORIZONTAL,
                              variable=self.filter_val,command=self.change_filter)
        ang_slider.grid(row=10, column=1, columnspan=2, sticky="we")

        # Error display at the very bottom
        error_label = tk.Label(self.root, textvariable=self.error_var,
                               fg="red", font=("Arial", 10))
        error_label.grid(row=11, column=0, columnspan=3, pady=(5,10))

    def change_filter(self,new_val):
        val = float(new_val)
        if val != self.old_filter_val:
            self.old_filter_val = val
            self.filter_pub.publish(Float64(data=val))


    def make_button(self, text, press_fn, row, col):
        btn = tk.Button(self.root, text=text, width=6, height=2)
        btn.grid(row=row, column=col, padx=5, pady=5)
        btn.bind("<ButtonPress>", lambda e: self.start_motion(press_fn))
        btn.bind("<ButtonRelease>", lambda e: self.stop_motion())
        return btn

    def set_cmd(self, linear=(0, 0, 0), angular=(0, 0, 0)):
        lin_coef = self.lin_scale_val.get()
        ang_coef = self.ang_scale_val.get()

        self.cmd = Twist()
        # scale the base 0.1/3 and 0.1 by the slider
        self.cmd.linear.x  = lin_coef * 2e-2 * linear[0]
        self.cmd.linear.y  = lin_coef * 2e-2 * linear[1]
        self.cmd.linear.z  = lin_coef * 2e-2 * linear[2]
        self.cmd.angular.x = ang_coef * 6e-2 * angular[0]
        self.cmd.angular.y = ang_coef * 6e-2 * angular[1]
        self.cmd.angular.z = ang_coef * 6e-2 * angular[2]


    def start_motion(self, cmd_fn):
        cmd_fn()
        self.active_cmd = self.cmd

    def stop_motion(self):
        self.cmd = Twist()
        self.cmd_pub.publish(self.cmd)
        self.active_cmd = None

    def error_callback(self, msg:Float64):
        self.error_var.set(f"Error: {msg.data*1e3:.2f} mm")

    def loop(self):
        if self.active_cmd and not rospy.is_shutdown():
            self.cmd_pub.publish(self.active_cmd)
        self.root.after(10, self.loop)

    def on_close(self):
        self.stop_motion()
        self.root.destroy()

if __name__ == "__main__":
    EEVelTeleopGUI().root.mainloop()
