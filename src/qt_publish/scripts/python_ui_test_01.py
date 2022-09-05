#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import tkinter as tk
from PIL import Image, ImageTk


window = tk.Tk()
window.title('window')

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("initial_pose")
        self.declare_parameter("robot_namespace", "robot1")
        self.declare_parameter("init_x_pose", 0.05)
        self.declare_parameter("init_y_pose", 0.5)
        self.x_pose = self.get_parameter("init_x_pose").value
        self.y_pose = self.get_parameter("init_y_pose").value
        self.alpha = math.pi/2.0  # radian value
        self.topic_name = "/" + self.get_parameter("robot_namespace").value + "/initialpose"

        self.pose_1_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/robot1/initialpose", 10)
        self.pose_2_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/robot2/initialpose", 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("initial_pose has been started.")

    def timer_callback(self):
        self.Publusher_1()
        self.Publusher_2()
    
    def Publusher_1(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 1.5
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        self.pose_1_publisher_.publish(pose_msg)
        self.get_logger().info("robot1 public initial_pose.")
    
    def Publusher_2(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = -1.64
        pose_msg.pose.pose.position.y = 1.47
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        self.pose_2_publisher_.publish(pose_msg)
        self.get_logger().info("robot2 public initial_pose.")

def create_button(txt):
    bt_1 = tk.Button(window, text=txt, bg='red', fg='white', font=('Arial', 12))
    bt_1['width'] = 50
    bt_1['height'] = 4
    bt_1['activebackground'] = 'red'        # 按鈕被按下的背景顏色
    bt_1['activeforeground'] = 'yellow'     # 按鈕被按下的文字顏色 ( 前景 )

    bt_1.grid(column=0, row=0)

def define_layout(obj, cols=1, rows=1):
    
    def method(trg, col, row):
        
        for c in range(cols):    
            trg.columnconfigure(c, weight=1)
        for r in range(rows):
            trg.rowconfigure(r, weight=1)

    if type(obj)==list:        
        [ method(trg, cols, rows) for trg in obj ]
    else:
        trg = obj
        method(trg, cols, rows)


def main(args=None):
    # rclpy.init(args=args)
    # node = NumberPublisherNode()
    # number_of_cycles = 5
    # spin_count = 0
    # while rclpy.ok() and spin_count < number_of_cycles:
    #     rclpy.spin_once(node, timeout_sec=1.0)
    #     spin_count += 1
    #     print('spin_count: ' + str(spin_count))
    # rclpy.shutdown()
    # print("publish end")
    
    """ window.geometry('500x100')
    lbl_1 = tk.Label(window, text='Hello World', bg='yellow', fg='#263238', font=('Arial', 12))
    lbl_1.grid(column=0, row=0)
    window.mainloop() """
    
    """ create_button('Button')
    window.mainloop() """
    
    align_mode = 'nswe'
    pad = 5

    div_size = 200
    img_size = div_size * 2
    div1 = tk.Frame(window,  width=img_size , height=img_size , bg='blue')
    div2 = tk.Frame(window,  width=div_size , height=div_size , bg='orange')
    div3 = tk.Frame(window,  width=div_size , height=div_size , bg='green')

    window.update()
    win_size = min( window.winfo_width(), window.winfo_height())
    print(win_size)

    div1.grid(column=0, row=0, padx=pad, pady=pad, rowspan=2, sticky=align_mode)
    div2.grid(column=1, row=0, padx=pad, pady=pad, sticky=align_mode)
    div3.grid(column=1, row=1, padx=pad, pady=pad, sticky=align_mode)

    define_layout(window, cols=2, rows=2)
    define_layout([div1, div2, div3])

    im = Image.open("/home/james/ros2_ws/src/qt_publish/scripts/Tory Motor.jpg")
    imTK = ImageTk.PhotoImage( im.resize( (img_size, img_size) ) )

    image_main = tk.Label(div1, image=imTK)
    image_main['height'] = img_size
    image_main['width'] = img_size

    image_main.grid(column=0, row=0, sticky=align_mode)

    lbl_title1 = tk.Label(div2, text='Hello', bg='orange', fg='white')
    lbl_title2 = tk.Label(div2, text="World", bg='orange', fg='white')

    lbl_title1.grid(column=0, row=0, sticky=align_mode)
    lbl_title2.grid(column=0, row=1, sticky=align_mode)

    bt1 = tk.Button(div3, text='Button 1', bg='green', fg='white')
    bt2 = tk.Button(div3, text='Button 2', bg='green', fg='white')
    bt3 = tk.Button(div3, text='Button 3', bg='green', fg='white')
    bt4 = tk.Button(div3, text='Button 4', bg='green', fg='white')

    bt1.grid(column=0, row=0, sticky=align_mode)
    bt2.grid(column=0, row=1, sticky=align_mode)
    bt3.grid(column=0, row=2, sticky=align_mode)
    bt4.grid(column=0, row=3, sticky=align_mode)

    bt1['command'] = lambda : get_size(window, image_main, im)

    define_layout(window, cols=2, rows=2)
    define_layout(div1)
    define_layout(div2, rows=2)
    define_layout(div3, rows=4)

    window.mainloop()

if __name__ == "__main__":
    main()
