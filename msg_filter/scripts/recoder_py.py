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

IMAGE_DIR         = "/home/stonewu/Data/image/animal/Sloth/"
WRENCH_NPY_DIR = "/home/stonewu/Data/6d_force/animal/Sloth/"
# WRENCH_CSV_DIR = "/home/stonewu/Data/Rectangular/"

count = 0

# 定义一个类来封装ros节点的逻辑
class DataNode(object):
    def __init__(self):
        # 初始化ros节点，命名为data_node
        rospy.init_node("recoder")
        # 创建三个订阅者，分别订阅image_data, imu_data, wrench_data主题，消息类型分别为Image, Imu, Wrench，回调函数分别为self.image_callback, self.imu_callback, self.wrench_callback
        self.image_sub = rospy.Subscriber("/rectify_crop_image", Image, self.image_callback)
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback)
        self.wrench_sub = rospy.Subscriber("/filted_wrench", WrenchStamped, self.wrench_callback)
        # 创建三个列表，分别用于缓存图像，惯性测量单元和力矩的数据
        self.image_buffer = []
        self.imu_buffer = []
        self.wrench_buffer = []
        # 定义一个常量，表示缓存的最大数目
        self.BUFFER_SIZE = 100


    def run(self):
        # 设置终端为原始模式，不需要回车即可获取终端输入
        tty.setraw(sys.stdin.fileno()) 
        # 循环执行，直到ros节点被关闭
        while not rospy.is_shutdown():
            # 获取终端输入的字符，如果没有输入，则返回空字符串
            key = sys.stdin.read(1)
            # 如果输入的字符是s，则保存最新的数据
            if key == "s":
                self.save_latest_data()
            # 如果输入的字符是a，则保存所有的数据
            elif key == "a":
                self.save_all_data()
            # 否则，打印一些提示信息
            else:
                rospy.loginfo("Press s to save the latest data or a to save all the data.")
    
    def save_latest_data(self):
        # 定义一个函数，用于保存最新的数据
        global count
        count = count + 1
        # 如果缓存列表为空，则打印一些警告信息
        if not self.image_buffer or not self.wrench_buffer:
            rospy.logwarn("No data to save.")
            return
        # 否则，获取缓存列表中的最后一个元素，即最新的数据
        image = self.image_buffer[-1]
        # imu = self.imu_buffer[-1]
        wrench = self.wrench_buffer[-1]
        # 生成文件名，使用当前时间作为前缀，使用png和csv作为后缀
        # prefix = rospy.get_time()
        prefix = str(count) # 把浮点数转换成字符串
        # prefix = prefix.replace(".", "") # 去掉时间戳中的小数点
        image_name = IMAGE_DIR + prefix + ".png"
        # imu_name = IMU_CSV_DIR + "_imu.csv"
        # wrench_name = WRENCH_CSV_DIR + "_wrench.csv"
        wrench_name = WRENCH_NPY_DIR + prefix + ".npy"
        # 保存图像数据为png文件，使用cv2库
        cv2.imwrite(image_name, image)
        # 保存惯性测量单元和力矩数据为csv文件，使用csv库
        # with open(imu_name, "a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(imu)
        # with open(wrench_name, "a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(wrench)
        np.save(wrench_name, wrench)
        # 打印一些提示信息
        rospy.loginfo("Saved the latest data as %s and %s." % (image_name, wrench_name))

    def image_callback(self, msg):
        # rospy.loginfo("image_callback")
        #print msg
        # 定义一个回调函数，用于处理图像数据
        # 将图像消息转换为numpy数组，使用cv_bridge库
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", image)
        key = cv2.waitKey(1)
        if key == ord('y'):
            #保存最新数据
            self.save_latest_data()
        if key == ord('q'):
            quit()
        # 将图像数据添加到缓存列表中
        self.image_buffer.append(image)
        
        # 打印缓存数量
        # print (len(self.image_buffer))

        # 如果缓存列表超过了最大数目，则删除最旧的数据
        if len(self.image_buffer) > self.BUFFER_SIZE:
            self.image_buffer.pop(0)

    def imu_callback(self, msg):
        # rospy.loginfo("imu_callback")
        # 定义一个回调函数，用于处理惯性测量单元数据
        # 将惯性测量单元消息转换为列表，包含线加速度，角速度和时间戳
        imu = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
               msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
               msg.header.stamp.to_sec()]
        # 将惯性测量单元数据添加到缓存列表中
        self.imu_buffer.append(imu)
        # 如果缓存列表超过了最大数目，则删除最旧的数据
        if len(self.imu_buffer) > self.BUFFER_SIZE:
            self.imu_buffer.pop(0)
    def wrench_callback(self, msg):
        # rospy.loginfo("wrench_callback")
        # 定义一个回调函数，用于处理力矩数据
        # 将力矩消息转换为列表，包含力和力矩的分量和时间戳
        wrenchstamped = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                         msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        # 将力矩数据添加到缓存列表中
        self.wrench_buffer.append(wrenchstamped)
        # 如果缓存列表超过了最大数目，则删除最旧的数据
        if len(self.wrench_buffer) > self.BUFFER_SIZE:
            self.wrench_buffer.pop(0)

    def save_all_data(self):
        # 定义一个函数，用于保存所有的数据
        # 如果缓存列表为空，则打印一些警告信息
        if not self.image_buffer or not self.imu_buffer or not self.wrench_buffer:
            rospy.logwarn("No data to save.")
            return
        # 否则，遍历缓存列表中的所有元素，分别保存为文件
        for i in range(len(self.image_buffer)):
            image = self.image_buffer[i]
            imu = self.imu_buffer[i]
            wrench = self.wrench_buffer[i]
            # 生成文件名，使用当前时间和索引作为前缀，使用png和csv作为后缀
            prefix = rospy.get_time()
            prefix = str(prefix) # 把浮点数转换成字符串
            prefix = prefix.replace(".", "") # 去掉时间戳中的小数点
            image_name = IMAGE_DIR + prefix + ".png"
            # imu_name = IMU_CSV_DIR + "_imu.csv"
            wrench_name = WRENCH_NPY_DIR + "_wrench.csv"
            # 保存图像数据为png文件，使用cv2库
            cv2.imwrite(image_name, image)
            # 保存惯性测量单元和力矩数据为csv文件，使用csv库
            # with open(imu_name, "a") as f:
            #     writer = csv.writer(f)
            #     writer.writerow(imu)
            with open(wrench_name, "a") as f:
                writer = csv.writer(f)
                writer.writerow(wrench)
            # 打印一些提示信息
            rospy.loginfo("Saved the data %d as %s, %s and %s." % (i, image_name, imu_name, wrench_name))
# 创建一个DataNode对象，并运行它
node = DataNode()
node.run()
