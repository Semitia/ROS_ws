import numpy as np
data = np.load("/home/stonewu/Data/6d_force/geometry/test/1.npy") # 读取npy文件
#取data前六个元素，即力矩的三个分量和力的三个分量
data2 = data[0:6]

print(data2) # 打印数据