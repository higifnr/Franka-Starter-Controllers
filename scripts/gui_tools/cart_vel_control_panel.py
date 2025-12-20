#!/usr/bin/env python3

import rospy
import tkinter as tk
from tkinter import ttk
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
import subprocess

class EEVelTeleopResponsive:
    def __init__(self):
        rospy.init_node("cartesian_velocity_teleop_gui_responsive")
        rospy.on_shutdown(self.shutdown)

        self.arm_id = rospy.get_param("/arm_id", "panda")
        self.work_frame = f"{self.arm_id}_EE"

        # Fixed publishers
        self.frame_pub = rospy.Publisher("/work_frame", String, queue_size=1)
        self.filter_pub = rospy.Publisher("/unified_velocity_controller/joint_velocity_filter", Float64, queue_size=1)

        # Command publisher
        self.cmd_pub = None
        self.cmd_pub_topic = None

        self.root = tk.Tk()
        self.root.title("Cartesian Velocity Teleop")
        self.root.geometry("600x400")  # initial size

        self.cmd = Twist()
        self.active_cmd = None

        self.lin_scale_val = tk.DoubleVar(value=1.0)
        self.ang_scale_val = tk.DoubleVar(value=1.0)
        self.old_filter_val = 0.1
        self.filter_val = tk.DoubleVar(value=self.old_filter_val)

        self.error_var = tk.StringVar(value="Error: N/A")

        self.controller_topics = []
        self.selected_topic = tk.StringVar()

        self.make_gui()

        rospy.Subscriber("/rcm_to_axis_error", Float64, self.error_callback)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Auto-populate controller topics
        self.populate_controller_topics()

        self.loop()

    # ------------------- Topic discovery -------------------
    def populate_controller_topics(self):
        try:
            output = subprocess.check_output(["rostopic", "list"]).decode("utf-8")
            all_topics = output.splitlines()
            filtered_topics = [t for t in all_topics if ("controller" in t.lower() or "command" in t.lower())]
            self.controller_topics = filtered_topics
            self.topic_box["values"] = filtered_topics
            if filtered_topics:
                self.selected_topic.set(filtered_topics[0])
                self.set_cmd_topic(filtered_topics[0])
            else:
                self.selected_topic.set("<no controller topics found>")
        except Exception as e:
            rospy.logwarn(f"Failed to list topics: {e}")
            self.selected_topic.set("<error listing topics>")

    def set_cmd_topic(self, topic_name):
        if not topic_name or topic_name == self.cmd_pub_topic:
            return
        if self.cmd_pub:
            self.cmd_pub.unregister()
        self.cmd_pub_topic = topic_name
        self.cmd_pub = rospy.Publisher(topic_name, Twist, queue_size=1)
        rospy.loginfo(f"Publishing Twist commands to: {topic_name}")

    # ------------------- GUI -------------------
    def make_gui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.columnconfigure(2, weight=1)
        for i in range(15):
            self.root.rowconfigure(i, weight=1)

        # Topic selection
        tk.Label(self.root, text="Controller Topic").grid(row=0, column=0, columnspan=2, sticky="w", padx=5, pady=5)
        self.topic_box = ttk.Combobox(self.root, textvariable=self.selected_topic, state="readonly")
        self.topic_box.grid(row=1, column=0, columnspan=2, sticky="we", padx=5, pady=5)
        tk.Button(self.root, text="Refresh Topics", command=self.populate_controller_topics).grid(row=1, column=2, sticky="we", padx=5, pady=5)
        self.topic_box.bind("<<ComboboxSelected>>", lambda e: self.set_cmd_topic(self.selected_topic.get()))

        # Frames for movement buttons
        trans_frame = tk.Frame(self.root)
        trans_frame.grid(row=2, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)
        trans_frame.columnconfigure([0,1,2], weight=1)
        trans_frame.rowconfigure([0,1,2], weight=1)

        # Cartesian translation buttons
        self.make_button("← X-", lambda: self.set_cmd(linear=(-1,0,0)), trans_frame, 1, 0)
        self.make_button("→ X+", lambda: self.set_cmd(linear=(1,0,0)), trans_frame, 1, 2)
        self.make_button("↓ Y-", lambda: self.set_cmd(linear=(0,-1,0)), trans_frame, 2, 1)
        self.make_button("↑ Y+", lambda: self.set_cmd(linear=(0,1,0)), trans_frame, 0, 1)
        self.make_button("⇣ Z-", lambda: self.set_cmd(linear=(0,0,-1)), trans_frame, 2, 0)
        self.make_button("⇡ Z+", lambda: self.set_cmd(linear=(0,0,1)), trans_frame, 2, 2)

        # Rotation buttons in separate frame
        rot_frame = tk.Frame(self.root)
        rot_frame.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)
        rot_frame.columnconfigure([0,1,2], weight=1)
        rot_frame.rowconfigure([0,1,2], weight=1)

        self.make_button("⟲ Rx-", lambda: self.set_cmd(angular=(-1,0,0)), rot_frame, 0, 0)
        self.make_button("⟳ Rx+", lambda: self.set_cmd(angular=(1,0,0)), rot_frame, 0, 2)
        self.make_button("⟱ Ry-", lambda: self.set_cmd(angular=(0,-1,0)), rot_frame, 1, 0)
        self.make_button("⟰ Ry+", lambda: self.set_cmd(angular=(0,1,0)), rot_frame, 1, 2)
        self.make_button("⤵ Rz-", lambda: self.set_cmd(angular=(0,0,-1)), rot_frame, 2, 0)
        self.make_button("⤴ Rz+", lambda: self.set_cmd(angular=(0,0,1)), rot_frame, 2, 2)

        # Sliders
        tk.Label(self.root,text="Linear Scale").grid(row=4, column=0, sticky="w", padx=5)
        tk.Scale(self.root, from_=0.0,to=2.0,resolution=0.01, orient=tk.HORIZONTAL, variable=self.lin_scale_val).grid(row=4,column=1,columnspan=2,sticky="we", padx=5)

        tk.Label(self.root,text="Angular Scale").grid(row=5, column=0, sticky="w", padx=5)
        tk.Scale(self.root, from_=0.0,to=2.0,resolution=0.01, orient=tk.HORIZONTAL, variable=self.ang_scale_val).grid(row=5,column=1,columnspan=2,sticky="we", padx=5)

        tk.Label(self.root,text="Filter parameter").grid(row=6, column=0, sticky="w", padx=5)
        tk.Scale(self.root, from_=0.0,to=1.0,resolution=0.01, orient=tk.HORIZONTAL, variable=self.filter_val, command=self.change_filter).grid(row=6,column=1,columnspan=2,sticky="we", padx=5)

        tk.Label(self.root,textvariable=self.error_var, fg="red", font=("Arial",10)).grid(row=7,column=0,columnspan=3, sticky="we", pady=5)

    # ------------------- Motion -------------------
    def make_button(self,text,fn,parent,row,col):
        btn = tk.Button(parent, text=text, font=("Arial", 12))
        btn.grid(row=row,column=col,sticky="nsew", padx=2, pady=2)
        btn.bind("<ButtonPress>", lambda e:self.start_motion(fn))
        btn.bind("<ButtonRelease>", lambda e:self.stop_motion())
        return btn

    def set_cmd(self, linear=(0,0,0), angular=(0,0,0)):
        lin=self.lin_scale_val.get()
        ang=self.ang_scale_val.get()
        self.cmd = Twist()
        self.cmd.linear.x=lin*2e-2*linear[0]
        self.cmd.linear.y=lin*2e-2*linear[1]
        self.cmd.linear.z=lin*2e-2*linear[2]
        self.cmd.angular.x=ang*6e-2*angular[0]
        self.cmd.angular.y=ang*6e-2*angular[1]
        self.cmd.angular.z=ang*6e-2*angular[2]

    def start_motion(self,fn):
        fn()
        self.active_cmd=self.cmd

    def stop_motion(self):
        self.cmd=Twist()
        if self.cmd_pub:
            self.cmd_pub.publish(self.cmd)
        self.active_cmd=None

    # ------------------- Filter & Error -------------------
    def change_filter(self,val):
        val=float(val)
        if val!=self.old_filter_val:
            self.old_filter_val=val
            self.filter_pub.publish(Float64(data=val))

    def error_callback(self,msg):
        self.error_var.set(f"Error: {msg.data*1e3:.2f} mm")

    # ------------------- Loop & Shutdown -------------------
    def loop(self):
        if self.active_cmd and self.cmd_pub and not rospy.is_shutdown():
            self.cmd_pub.publish(self.active_cmd)
        self.root.after(10,self.loop)

    def shutdown(self):
        try:
            self.root.quit()
            self.root.destroy()
        except:
            pass

    def on_close(self):
        self.stop_motion()
        self.root.destroy()

if __name__=="__main__":
    EEVelTeleopResponsive().root.mainloop()
