#include "IMU.h"
#include "mymath.h"
#include "math.h"
#include "time.h"
#include "usart.h"

// ==================================================================================
// ����:
// ���붨��'halfT 'Ϊ���ڵ�һ�룬�Լ��˲����Ĳ���Kp��Ki
// ��Ԫ��'q0', 'q1', 'q2', 'q3'����Ϊȫ�ֱ���
// ��Ҫ��ÿһ���������ڵ���'IMUupdate()'����
// ���������ݵ�λ�ǻ���/�룬���ٶȼƵĵ�λ�޹���Ҫ����Ϊ�ᱻ�淶��


// ==================================================================================
#define Kp 	2.0f    // ��������
#define Ki 	0.005f  // ���ֳ���
float halfT;				//������
// ==================================================================================


// ��������
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     	// ��ʼ����Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;    	// �������ۼ�ֵ 

// ==================================================================================
// ����ԭ�ͣ�void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz) 
// ��        �ܣ�������Ԫ����̬���� + �����ƻ����˲�
// ��        �룺���������ݼ����ٶȼ�����
// ==================================================================================
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx, float my, float mz) 
{
	float init_gz = gz;
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
	////������ٶȴ�������������ݾ�Ϊ0������Ըü��ٶ����ݣ� ��ʹ����һ����̬����Ľ���������ڼ��ٶ����ݹ�һ�������ʱ�򣬻ᵼ�³���0�Ĵ���
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
	//�Ѽ��ٶȼƵ���ά����ת�ɵ�λ��������һ����
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//�����������ٶȷ����ڷ���������ϵ�еı�ʾ��Ϊ��Ԫ����ʾ����ת����ĵ�����
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//���ٶȼƶ�ȡ�ķ������������ٶȷ���Ĳ�ֵ����������˼���
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;

	//����ۻ���������ֳ������
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//�ò���������PI����������ƫ�����������ݶ����е�ƫ����	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	halfT = Get_IMU_Update_Time() / 2.0f;

	//һ�׽����㷨
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz)*halfT;
	q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy)*halfT;
	q2 = q2+ ( q0 * gy - q1 * gz + q3 * gx)*halfT;
	q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx)*halfT; 

	//��Ԫ���淶��
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	float Pitch = asin(-2 * q1 * q3 + 2 * q0* q2);			//����pitch����λΪ���ȣ���y����ת
	float Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);		//����roll����λΪ���ȣ���x����ת

	out_angle.pitch = Pitch * Radian_to_Angle;			
	out_angle.roll = -Roll * Radian_to_Angle;
	//out_angle.yaw = 0.9f * (out_angle.yaw - init_gz*2*halfT*Radian_to_Angle) + 0.1f * Radian_to_Angle * atan2(my*cos(Roll) + mz*sin(Roll), mx*cos(Pitch) + my*sin(Pitch)*sin(Roll)-mz*cos(Roll)*sin(Pitch));
	
	if(filter_gyro.z >=80 || filter_gyro.z <= -80)
		out_angle.yaw += -filter_gyro.z * RawData_to_Angle * 2 * halfT;
	}	
}






/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

/*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
   //auxiliary variables to reduce number of repeated operations��
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
          
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
   //compute reference direction of flux
   hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);

/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣����Ҷ���byָ������������by=ĳֵ��bx=0
������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
��Ϊbx=0�����Ծͼ򻯳�(by*by)  = ((hx*hx) + (hy*hy))�������by�������޸�by��bxָ����Զ����ĸ���ָ������*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
��Ϊbx=0�����������漰��bx�Ĳ��ֶ���ʡ���ˡ�ͬ��by=0�����������漰��by�Ĳ���Ҳ���Ա�ʡ�ԣ�������Լ������Ǹ���ָ���йء�
������������vxyz�����㣬��Ϊ����g��az=1��ax=ay=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
����Կ���������ʽ��wxyz�Ĺ�ʽ����by����ay��0������bz����az��1�����ͱ����vxyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5f - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5f - q1q1 - q2q2);
           
//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
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

   halfT = Get_IMU_Update_Time() / 2.0f;		//�õ�ÿ����̬���µ����ڵ�һ��
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //�ܹؼ���һ�仰��ԭ�㷨û��
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //���Բ������ڵ�һ��
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise����Ԫ�������㷨
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
        
/*Y��ָ������������Ԫ�������Pitch  Roll  Yaw��ֻ������ҪPID����ʱ�Ž���Ԫ��ת��Ϊŷ����
����57.295780��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	 
	//Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //ƫ���ǣ���z��ת��	
	//Pitch =  asin(2*q2*q3 + 2*q0*q1) * 57.295780; //�����ǣ���x��ת��	 
    //Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //�����ǣ���y��ת��

/*���������Ԫ�������Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
����57.295780��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	
	 out_angle.pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.295780; //�����ǣ���y��ת��	 
   out_angle.roll  = -atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.295780; //�����ǣ���x��ת��
	 out_angle.yaw   = -atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.295780;  //ƫ���ǣ���z��ת��

}








