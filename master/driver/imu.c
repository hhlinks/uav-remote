#include "imu.h"
#include "imath.h"
#include "math.h"
#include "mpu6050.h"
#include "timer.h"
#include "nav.h"

_Matrix Mat = {0};

_Attitude att = {0};

#define kp 	    0.5f        //比例增益决定加速度计/磁力计的收敛速率
#define ki 	    0.0001f     //积分增益决定陀螺仪偏差的收敛速率

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     //表示估计方向的四元数元素
float errIntX = 0, errIntY = 0, errIntZ = 0;    //误差积分

_Time_test att_time;

//姿态解算
void IMUUpdate(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ) 
{
	float norm;
	float gbX, gbY, gbZ;
	float errX, errY, errZ;
 
  if(accX*accY*accZ==0)
      return;
    
  //姿态解算时间检测
  time_check(&att_time);
    
  //加速度计数据归一化成单位矢量
	norm = FastSqrt(accX*accX + accY*accY + accZ*accZ);
	accX = accX * norm;
	accY = accY * norm;
	accZ = accZ * norm;
	
	//计算机体重力向量
  gbX = Mat.DCM_T[0][2];
  gbY = Mat.DCM_T[1][2];
  gbZ = Mat.DCM_T[2][2];
	
	//叉乘得到误差
  errX = accY*gbZ - accZ*gbY;
	errY = accZ*gbX - accX*gbZ;
	errZ = accX*gbY - accY*gbX;
 
  //对误差向量进行积分
	errIntX = errIntX + errX*ki;
	errIntY = errIntY + errY*ki;
	errIntZ = errIntZ + errZ*ki;

  //通过Kp、Ki两个参数来调节加速度计传感器数据与陀螺仪传感器数据之前的权重。
	gyroX = gyroX + kp*errX + errIntX;
	gyroY = gyroY + kp*errY + errIntY;
	gyroZ = gyroZ + kp*errZ + errIntZ;

  //一阶龙格库塔法更新四元数 
	q0 = q0 + (-q1*gyroX - q2*gyroY - q3*gyroZ)* att_time.delta_time_ms*0.0005f;
	q1 = q1 + ( q0*gyroX + q2*gyroZ - q3*gyroY)* att_time.delta_time_ms*0.0005f;
	q2 = q2 + ( q0*gyroY - q1*gyroZ + q3*gyroX)* att_time.delta_time_ms*0.0005f;
	q3 = q3 + ( q0*gyroZ + q1*gyroY - q2*gyroX)* att_time.delta_time_ms*0.0005f; 

  //把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
	norm = FastSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
   
	//四元数转换成欧拉角
	att.pit = atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * RAD_TO_DEG;
	att.rol = asin(2.0f*(q0*q2 - q1*q3)) * RAD_TO_DEG;       
  att.yaw += Mpu.deg_s.z  * att_time.delta_time_ms*0.001f;	//z轴角速度积分得偏航角 

}

//旋转矩阵：机体坐标系 -> 地理坐标系
void rotation_matrix(void)
{
    Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);

    Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
    Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
    Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);
     
    Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
    Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
    Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}


//旋转矩阵的转置矩阵：地理坐标系 -> 机体坐标系
void rotation_matrix_T(void)
{
    Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
    Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);    
    Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2); 
    
    Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
    Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;  
    Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);    
    
    Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
    Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
    Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}

float sin_pit = 0,sin_rol = 0,sin_yaw = 0;
float cos_pit = 0,cos_rol = 0,cos_yaw = 0;

void Matrix_ready(void)
{
    rotation_matrix();                      //旋转矩阵更新
    rotation_matrix_T();                    //旋转矩阵的逆矩阵更新
    
		//通过欧拉角转换成方向余弦的每个元素
    sin_pit = sin(att.pit * deg_to_rad);
    cos_pit = cos(att.pit * deg_to_rad);
    sin_rol = sin(att.rol * deg_to_rad);
    cos_rol = cos(att.rol * deg_to_rad);
    sin_yaw = sin(att.yaw * deg_to_rad);
    cos_yaw = cos(att.yaw * deg_to_rad);     

    //此处输入参数为三轴加速度数据，以便进行数据转换
		get_nav_acc(acc_fix_lpf);       //导航系下三轴加速度  cm/s^2
    //sins_high();                  //竖直方向惯导融合
}
