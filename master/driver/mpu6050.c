#include "mpu6050.h"
#include "iic.h"
#include "systick.h"
#include "acc_cal.h"

S16_XYZ accRaw = {0};                  //加速度计原始数据
S16_XYZ gyroRaw = {0};                 //陀螺仪原始数据
SI_F_XYZ accButterworthData = {0};		//加速度计巴特沃斯低通滤波后的数据
SI_F_XYZ gyroButterworthData = {0};		//陀螺仪巴特沃斯低通滤波后的数据


SI_F_XYZ acc_att_lpf = {0};
SI_F_XYZ acc_fix_lpf = {0};

SI_F_XYZ acc_1_lpf = {0};
SI_F_XYZ acc_butter_lpf = {0};

SI_F_XYZ gyro_lpf = {0};
SI_F_XYZ gyro_offset = {0,0,0};		//陀螺仪零偏数据

_Mpu6050_data Mpu = {0};

//mpu初始化
void mpu6050_init(void)
{
	IIC_Write_One_Byte(0xD0,PWR_MGMT_1, 0x80);
	delay_ms(100);
	IIC_Write_One_Byte(0xD0,PWR_MGMT_1, 0x00);		//唤醒mpu

    /* when DLPF is disabled( DLPF_CFG=0 or 7),陀螺仪输出频率 = 8kHz;
       when DLPFis enabled,陀螺仪输出频率 = 1KHz
       fs(采样频率) = 陀螺仪输出频率 / (1 + SMPLRT_DIV)*/

	IIC_Write_One_Byte(0xD0,SMPLRT_DIV, 0x00);		        //sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
	IIC_Write_One_Byte(0xD0,MPU_CONFIG, 0x03);              //内部低通  acc:44hz	gyro:42hz
	IIC_Write_One_Byte(0xD0,GYRO_CONFIG, 0x18);			    // gyro scale  ：+-2000deg/s
	IIC_Write_One_Byte(0xD0,ACCEL_CONFIG, 0x10);			// Accel scale ：+-8g (65536/16=4096 LSB/g)
}

//两字节数据合成
static int GetData(unsigned char REG_Address)
{
	unsigned char H,L;
	H = IIC_Read_One_Byte(0xD0,REG_Address);
	L = IIC_Read_One_Byte(0xD0,REG_Address+1);

	return ((H<<8)+L);
}

//get id
uint8_t get_mpu_id(void)
{
    u8 mpu_id;
    mpu_id = IIC_Read_One_Byte(0xD0,WHO_AM_I);

    return mpu_id;
}

//读取加速度计原始三轴数据
void GetAccRaw(void)
{
    accRaw.x = GetData(ACCEL_XOUT_H);
    accRaw.y = GetData(ACCEL_YOUT_H);
    accRaw.z = GetData(ACCEL_ZOUT_H);

	//椭球校准后的三轴加速度量
	accButterworthData.x = (float)(cal_acc.K[0]*((float)accRaw.x) - cal_acc.B[0]*one_g_to_acc);
	accButterworthData.y = (float)(cal_acc.K[1]*((float)accRaw.y) - cal_acc.B[1]*one_g_to_acc);
	accButterworthData.z = (float)(cal_acc.K[2]*((float)accRaw.z) - cal_acc.B[2]*one_g_to_acc);
}

_Butterworth_parameter gyro_30hz_parameter =
{
    //200hz---30hz
    1,  -0.7477891782585,    0.272214937925,
    0.1311064399166,   0.2622128798333,   0.1311064399166
};

_Butterworth_data   gyroButterData[3];

//二阶巴特沃斯低通滤波
float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter)
{
    buffer->input_data[2] = now_input;

    /* Butterworth LPF */
    buffer->output_data[2] =   parameter->b[0] * buffer->input_data[2]
                             + parameter->b[1] * buffer->input_data[1]
                             + parameter->b[2] * buffer->input_data[0]
                             - parameter->a[1] * buffer->output_data[1]
                             - parameter->a[2] * buffer->output_data[0];
    /* x(n) 保存 */
    buffer->input_data[0] = buffer->input_data[1];
    buffer->input_data[1] = buffer->input_data[2];
    /* y(n) 保存 */
    buffer->output_data[0] = buffer->output_data[1];
    buffer->output_data[1] = buffer->output_data[2];

    return buffer->output_data[2];
}

//读取陀螺仪三轴数据量
void GetGyroRaw(void)
{
    gyroRaw.x = GetData(GYRO_XOUT_H) - gyro_offset.x;		//原始数据
    gyroRaw.y = GetData(GYRO_YOUT_H) - gyro_offset.y;
    gyroRaw.z = GetData(GYRO_ZOUT_H) - gyro_offset.z;

    gyroButterworthData.x = (float)butterworth_lpf(((float)gyroRaw.x),&gyroButterData[0],&gyro_30hz_parameter);		//巴特沃斯低通滤波后的数据
    gyroButterworthData.y = (float)butterworth_lpf(((float)gyroRaw.y),&gyroButterData[1],&gyro_30hz_parameter);
    gyroButterworthData.z = (float)butterworth_lpf(((float)gyroRaw.z),&gyroButterData[2],&gyro_30hz_parameter);
}


//求取IIR滤波因子
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
	*out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}

//加速度IIR低通滤波
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor)
{
	acc_out->x = acc_out->x + lpf_factor*(acc_in->x - acc_out->x);
	acc_out->y = acc_out->y + lpf_factor*(acc_in->y - acc_out->y);
	acc_out->z = acc_out->z + lpf_factor*(acc_in->z - acc_out->z);
}

//加速度计滤波参数
_Butterworth_parameter acc_5hz_parameter =
{
    1,                  -1.778631777825,    0.8008026466657,
    0.005542717210281,   0.01108543442056,  0.005542717210281
};

_Butterworth_data   acc_butter_data[3];

//加速度计巴特沃斯低通
void acc_butterworth_lpf(SI_F_XYZ *accIn,SI_F_XYZ *accOut)
{
    accOut->x = butterworth_lpf(accIn->x,&acc_butter_data[0],&acc_5hz_parameter);
    accOut->y = butterworth_lpf(accIn->y,&acc_butter_data[1],&acc_5hz_parameter);
    accOut->z = butterworth_lpf(accIn->z,&acc_butter_data[2],&acc_5hz_parameter);
}

//原始加速度量转为 g
void AccDataTransToG(SI_F_XYZ *accIn,SI_F_XYZ *accOut)
{
	accOut->x = (float)(accIn->x * acc_raw_to_g);
	accOut->y = (float)(accIn->y * acc_raw_to_g);
	accOut->z = (float)(accIn->z * acc_raw_to_g);
}

//滤波后的数据转成（弧度/秒）单位
void RadTransform(SI_F_XYZ *gyroIn,SI_F_XYZ *gyroRadOut)
{
	gyroRadOut->x = (float)(gyroIn->x * gyro_raw_to_radian_s);
	gyroRadOut->y = (float)(gyroIn->y * gyro_raw_to_radian_s);
	gyroRadOut->z = (float)(gyroIn->z * gyro_raw_to_radian_s);
}

//滤波后的数据转成（度/秒）单位
void DegTransform(SI_F_XYZ *gyroIn,SI_F_XYZ *gyroDegOut)
{
	gyroDegOut->x = (float)(gyroIn->x * gyro_raw_to_deg_s);
	gyroDegOut->y = (float)(gyroIn->y * gyro_raw_to_deg_s);
	gyroDegOut->z = (float)(gyroIn->z * gyro_raw_to_deg_s);
}
