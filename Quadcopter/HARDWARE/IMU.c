#include "IMU.h"
#include "mymath.h"
#include "math.h"
#include "time.h"
#include "usart.h"

// ==================================================================================
// 描述:
// 必须定义'halfT '为周期的一半，以及滤波器的参数Kp和Ki
// 四元数'q0', 'q1', 'q2', 'q3'定义为全局变量
// 需要在每一个采样周期调用'IMUupdate()'函数
// 陀螺仪数据单位是弧度/秒，加速度计的单位无关重要，因为会被规范化


// ==================================================================================
#define Kp 	2.0f    // 比例常数
#define Ki 	0.005f  // 积分常数
float halfT;				//半周期
// ==================================================================================


// 变量定义
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     	// 初始化四元数
float exInt = 0, eyInt = 0, ezInt = 0;    	// 误差积分累计值 

// ==================================================================================
// 函数原型：void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz) 
// 功        能：六轴四元数姿态解算 + 磁力计互补滤波
// 输        入：陀螺仪数据及加速度计数据
// ==================================================================================
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz) 
{
	float init_gz = gz;
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
	////如果加速度传感器各轴的数据均为0，则忽略该加速度数据， 且使用上一次姿态解算的结果，否则在加速度数据归一化处理的时候，会导致除以0的错误
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
	//把加速度计的三维向量转成单位向量（归一化）
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//估计重力加速度方向在飞行器坐标系中的表示，为四元数表示的旋转矩阵的第三行
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//加速度计读取的方向与重力加速度方向的差值，用向量叉乘计算
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;

	//误差累积，已与积分常数相乘
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	halfT = Get_IMU_Update_Time() / 2.0f;

	//一阶近似算法
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz)*halfT;
	q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy)*halfT;
	q2 = q2+ ( q0 * gy - q1 * gz + q3 * gx)*halfT;
	q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx)*halfT; 

	//四元数规范化
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	float Pitch = asin(-2 * q1 * q3 + 2 * q0* q2);			//计算pitch，单位为弧度，绕y轴旋转
	float Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);		//计算roll，单位为弧度，绕x轴旋转

	out_angle.pitch = Pitch * Radian_to_Angle;			
	out_angle.roll = -Roll * Radian_to_Angle;
	//out_angle.yaw = 0.9f * (out_angle.yaw - init_gz*2*halfT*Radian_to_Angle) + 0.1f * Radian_to_Angle * atan2(my*cos(Roll) + mz*sin(Roll), mx*cos(Pitch) + my*sin(Pitch)*sin(Roll)-mz*cos(Roll)*sin(Pitch));
	
	if(filter_gyro.z >=80 || filter_gyro.z <= -80)
		out_angle.yaw += -filter_gyro.z * RawData_to_Angle * 2 * halfT;
	}	
}






/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

/*方便之后的程序使用，减少计算时间*/
   //auxiliary variables to reduce number of repeated operations，
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
          
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
   //compute reference direction of flux
   hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);

/*计算地理坐标系下的磁场矢量bxyz（参考值）。
因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），我定义by指向正北，所以by=某值，bx=0
但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。这里修改by和bx指向可以定义哪个轴指向正北*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
因为bx=0，所以所有涉及到bx的部分都被省略了。同理by=0，所以所有涉及到by的部分也可以被省略，这根据自己定义那个轴指北有关。
类似上面重力vxyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：wxyz的公式，把by换成ay（0），把bz换成az（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5f - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5f - q1q1 - q2q2);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
//   // integral error scaled integral gain
//   exInt = exInt + ex*Ki;		
//   eyInt = eyInt + ey*Ki;
//   ezInt = ezInt + ez*Ki;
//   // adjusted gyroscope measurements
//   gx = gx + Kp*ex + exInt;
//   gy = gy + Kp*ey + eyInt;
//   gz = gz + Kp*ez + ezInt;

   halfT = Get_IMU_Update_Time() / 2.0f;		//得到每次姿态更新的周期的一半
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //乘以采样周期的一半
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise，四元数更新算法
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
   // normalise quaternion
   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 * norm;       //w
   q1 = q1 * norm;       //x
   q2 = q2 * norm;       //y
   q3 = q3 * norm;       //z
        
/*Y轴指向正北，由四元数计算出Pitch  Roll  Yaw，只需在需要PID控制时才将四元数转化为欧拉角
乘以57.295780是为了将弧度转化为角度*/
	 
	//Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //偏航角，绕z轴转动	
	//Pitch =  asin(2*q2*q3 + 2*q0*q1) * 57.295780; //俯仰角，绕x轴转动	 
    //Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //滚动角，绕y轴转动

/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.295780是为了将弧度转化为角度*/
	
	 out_angle.pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.295780; //俯仰角，绕y轴转动	 
   out_angle.roll  = -atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.295780; //滚动角，绕x轴转动
	 out_angle.yaw   = -atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.295780;  //偏航角，绕z轴转动

}








