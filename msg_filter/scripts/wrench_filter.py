# 导入ros相关的库
import rospy
from sensor_msgs.msg import Image, Imu # 用于订阅图像和惯性测量单元的消息类型
from geometry_msgs.msg import WrenchStamped # 用于订阅力矩的消息类型
from cv_bridge import CvBridge # 用于图像转换的库
import cv2 # 用于图像处理的库
import numpy as np # 用于数值计算的库
import csv # 用于保存csv文件的库
import sys # 用于获取终端输入的库
import tty # 用于获取终端输入的库

startflag = 0
count = 0
# 定义一个类来封装ros节点的逻辑
class DataNode(object):
    def __init__(self):
        # 初始化ros节点，命名为data_node
        rospy.init_node("filter")
        # 创建三个订阅者，分别订阅image_data, imu_data, wrench_data主题，消息类型分别为Image, Imu, Wrench，回调函数分别为self.image_callback, self.imu_callback, self.wrench_callback
        self.wrench_sub = rospy.Subscriber("/bus0/ft_sensor0/ft_sensor_readings/wrench", WrenchStamped, self.wrench_callback)
        self.wrench_buffer = []
        # 定义一个常量，表示缓存的最大数目
        self.BUFFER_SIZE = 100
        #定义一个变量，用来存储wrench数据的偏置值
        self.wrench_bias = [0, 0, 0, 0, 0, 0]
        #定义一个发布者，用来发布减去偏置值的wrench数据
        self.wrench_pub = rospy.Publisher("/filted_wrench", WrenchStamped, queue_size=10)

    def run(self):
        # 设置终端为原始模式，不需要回车即可获取终端输入
        tty.setraw(sys.stdin.fileno()) 
        # 循环执行，直到ros节点被关闭
        while not rospy.is_shutdown():
            # 获取终端输入的字符，如果没有输入，则返回空字符串
            key = sys.stdin.read(1)
            # 如果输入的字符是f，则保存最新的数据
            if key == "f":
                global startflag, count
                count = 0
                startflag = 1;
            # 否则，打印一些提示信息
            else:
                rospy.loginfo("Press f to update the wrench bias.")
    
    def wrench_callback(self, msg):
        global startflag
        if startflag != 1:
            #发布减去偏置值的wrench数据
            new_wrench = msg
            new_wrench.wrench.force.x = msg.wrench.force.x - self.wrench_bias[0]
            new_wrench.wrench.force.y = msg.wrench.force.y - self.wrench_bias[1]
            new_wrench.wrench.force.z = msg.wrench.force.z - self.wrench_bias[2]
            new_wrench.wrench.torque.x = msg.wrench.torque.x - self.wrench_bias[3]
            new_wrench.wrench.torque.y = msg.wrench.torque.y - self.wrench_bias[4]
            new_wrench.wrench.torque.z = msg.wrench.torque.z - self.wrench_bias[5]
            self.wrench_pub.publish(new_wrench)
        else:
            #startflag==1时，采集数据更新偏置值
            global count
            count = count + 1
            wrenchstamped = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
            # 将力矩数据添加到缓存列表中
            self.wrench_buffer.append(wrenchstamped)
            # 如果缓存列表超过了最大数目，则删除最旧的数据
            if len(self.wrench_buffer) > self.BUFFER_SIZE:
                self.wrench_buffer.pop(0)
            if count > self.BUFFER_SIZE:
                #偏执值为缓存列表中的平均值
                self.wrench_bias = np.mean(self.wrench_buffer, axis=0)
                #打印偏置值
                rospy.loginfo("Bias: %f, %f, %f, %f, %f, %f", self.wrench_bias[0], self.wrench_bias[1], self.wrench_bias[2], self.wrench_bias[3], self.wrench_bias[4], self.wrench_bias[5])
                #将count和startflag置0
                count = 0
                startflag = 0
                rospy.loginfo("Bias updated.")

        

# 创建一个DataNode对象，并运行它
node = DataNode()
node.run()
