
# JetsonNano视觉跟随小车

> 运行环境: 英伟达 - JetsonNano  Ubuntu18.04 - Arm64    
视觉处理部分: OpenCV4.5 + Tensorflow   
小车控制系统: ROS-melodic   
开发环境: VScode - C++  
文中如有纰漏请联系作者：AndyJen 联系邮箱： 1260105099@qq.com

> **官方文档:**   
[OpenCV](github.com/opencv/opencv/wiki)     
[ROS](wiki.ros.org)     
[Tensorflow](https://tensorflow.google.cn/)     

基本环境搭建---参考: 
[10天零基础玩转ROS小车](https://blog.csdn.net/qq_37510774/article/details/106950332#comments_15975483)

由于ROS的作用是控制小车的，所以这里直接采用外部接口的方式控制
通过发布和订阅小车话题，就可以实现小车追踪任务

## Part.1 开发环境测试
1. 视觉处理的第一部分首先需要打开摄像头，
在Ubuntu中，创建camera.py文件用于摄像头操作与Opencv处理功能


```
cd catkin_ws
touch camera.py tensorflow.py
```

创建文件后，在camera.py中写入以下代码测试OpenCV是否安装成功：
```
import numpy as np
import cv2 as cv

class camera_drv:
    def __init__(self):
        self.model = 0

cap = cv.VideoCapture(0)
while(1):
    # get a frame
    ret, frame = cap.read()
    # show a frame
    cv.imshow("capture", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv.destroyAllWindows()

if __name__ == '__main__':
    m_cmr = camera_drv()
    print(cv.__version__)
    print("\nCreate camera model...\n")
```
tensorflow.py中写入以下代码测试Tensorflow是否安装成功：
```
import tensorflow as tf 

mnist = tf.keras.datasets.mnist

(x_train, y_train),(x_test, y_test) = mnist.load_data()
x_train, x_test = x_train / 255.0, x_test / 255.0

model = tf.keras.models.Sequential([
  tf.keras.layers.Flatten(input_shape=(28, 28)),
  tf.keras.layers.Dense(128, activation='relu'),
  tf.keras.layers.Dropout(0.2),
  tf.keras.layers.Dense(10, activation='softmax')
])

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

model.fit(x_train, y_train, epochs=5)
model.evaluate(x_test, y_test)


if __name__ == '__main__':

    print("\nCreate Tensorflow model...\n")
```


* 虚拟机Camera黑屏问题：将虚拟机的USB兼容性调至你的Camera USB类型就可以


## Part.2 位置跟踪定位输出
代码位置: http://github.com/Andy-Jen/catkin_ws
进入工作空间下载代码
```
cd ~/catkin_ws
git clone "https://github.com/Andy-Jen/catkin_ws.git"
```
编译代码
```
cd ~/catkin_ws
catkin_make
```
开启外设节点
```
终端1:
roslaunch mbsrobot bringup.launch 

终端2:
CSI摄像头:
rosrun csi_camera csi_camera
USB摄像头: 
rosrun uvc_camera uvc_camera

终端3:
rosrun track_ws kcf_node
```
打开节点后会弹出Camera界面
在界面选择跟踪目标后，节点会根据目标特征矩阵对实时图像进行卷积操作，计算特征值后输出图像位置。  
#### 1. 算法原理
核相关滤波算法，Correlation Filter应用于tracking方面最朴素的想法就是：相关是衡量两个信号相似值的度量，如果两个信号越相似，那么其相关值就越高，而在tracking的应用里，就是需要设计一个滤波模板，使得当它作用在跟踪目标上时，得到的响应最大，最大响应值的位置就是目标的位置。  
#### 2. 循环矩阵
一维的情况下就是矩阵相乘的问题了，就是矩阵分析当中学过的左乘一个单位矩阵和右乘一个单位矩阵。左乘是行变换，右乘列变化。目的就是得到更多的样本，每乘一次都是一个新的样本，这样的话就可以多出来n*n个样本了，这就是循环矩阵在这里最大的用处，制造样本的数量，以图像的形式展示就是这样的，一个样本经过循环矩阵之后就可以产生这么多的样本。    
把图像向上、向下分别移动不同的像素得到新的样本图像，这就是循环之后的样本，这样算是直接增加了样本的数量，然后用来对分类器进行训练，更多的样本肯定能够训练的分类器的效果就更好了。

#### 3. 样本跟踪
核算法作为输出节点，将特征位置映射为ROS小车速度和方向话题，通过runtrack.cpp发布，滤除噪声放大速度增益后输出控制小车运动，通过增量式PID控制器调节运动前后的停顿。

#### 4. 节点简析
**bringup.launch:**    
默认底盘节点，开启后JetsonNano通过串口与底盘通信，底盘运行rosserial节点获取JetsonNano发布的运动向量话题，通过底盘控制板进行运动控制

**uvc_camera/csi_camera**   
摄像机节点，开启摄像机并发布image话题

**kcf_node**    
核相关滤波算法节点，通过采集camera发布的image话题，提取图像特征识别相应位置后返回位置信息，判断跟随运动方向和速度，经过PID控制器后输出速度矢量话题。

