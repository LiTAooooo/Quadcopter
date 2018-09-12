# 基于STM32F401RE的四旋翼飞行器设计

## 项目背景
该项目为电子科技大学信息与软件工程学院嵌入式专业综合设计课题

## 主要硬件
- 主控制器：Nucleo-STM32F401RE
- 姿态解算模块：GY-86（内含MPU6050,HMC5883L,MS5611）
- 无刷电机：MT2213 935KV
- 电调：天行者20A
- 遥控器：FS-I6
- 接收机：FS-IA6B
- 机架：F450

## 通信与上位机
- 串口通信：HC-05蓝牙模块（主、从）
- 上位机：匿名地面站

## 关键流程
- 遥控器输入捕获（PWM信号处理）
- 上位机通信（串口通信协议）
- 姿态解算（IMU算法）
- PID控制（串级PID）
- PID参数整定

## 项目文件
- 本项目的工程文件位于[Quadcopter](Quadcopter)目录下，可使用`keil`打开位于[USER](Quadcopter/USER)目录下的工程
- 详细说明请参考`综合设计Ⅲ四轴飞行器设计.docx`以及`四轴综合设计Ⅲ答辩.pptx`

## 成果展示
- 实物展示：

  ![](https://litaooooo.github.io/page-examples/Quadcopter.jpg)

- 视频展示：

  1. [PID参数整定](https://v.youku.com/v_show/id_XMzgyMDU3MTA5Mg==.html)
  
  2. [试飞视频](https://v.youku.com/v_show/id_XMzMxNzY2NzA3Ng==.html)
