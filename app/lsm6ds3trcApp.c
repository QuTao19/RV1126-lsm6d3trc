#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include "lsm6ds3trcApp.h"
#include "math.h"

#define Kp 		300.0f				/* 决定加速度计收敛速度 */
#define Ki 		0.02f				/* 决定陀螺仪偏差的收敛速度 */
#define halfT	1.0f/(2*833.0f)		/* 833HZ */
#define dt		1.0f/833.0f			/* 833HZ对应周期 */
#define pi 		3.1415926f			

/* 姿态解算 */
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;
float Pitch_zyx, Roll_zyx, Yaw_zyx;
float Pitch_zxy, Roll_zxy, Yaw_zxy;
float Yaw, Pitch, Roll;

/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */

int main(int argc, char *argv[])
{
	int fd;
	char *filename;	
	char databuf[14];

	/* 原始数据处理 */
	int acc_x_reg, acc_y_reg, acc_z_reg;
	int dec_acc_x, dec_acc_y, dec_acc_z;
	float acc_x, acc_y, acc_z;
	int gyr_x_reg, gyr_y_reg, gyr_z_reg;
	int dec_gyr_x, dec_gyr_y, dec_gyr_z;
	float gyr_x, gyr_y, gyr_z;
	int temp_reg;
	int dec_temp;
	float temp;	

	int ret = 0;

	if (argc != 2) {
		printf("Error Usage!\r\n");
		return -1;
	}

	filename = argv[1];
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}

	while (1) {
		ret = read(fd, databuf, sizeof(databuf));
		if(ret == 0) { 			/* 数据读取成功 */

			/* 加速度raw数据 */
			acc_x_reg = (databuf[1] << 8) | databuf[0];
			acc_y_reg = (databuf[3] << 8) | databuf[2];
			acc_z_reg = (databuf[5] << 8) | databuf[4];

			/* 补码转原码 */
			dec_acc_x = complementToOriginal(acc_x_reg);
			dec_acc_y = complementToOriginal(acc_y_reg);
			dec_acc_z = complementToOriginal(acc_z_reg);

			/* 灵敏度转换 */
			acc_x = ((float)dec_acc_x * ACC_FSXL_2G_scale)/1000;	/* 单位g */
			acc_y = ((float)dec_acc_y * ACC_FSXL_2G_scale)/1000;
			acc_z = ((float)dec_acc_z * ACC_FSXL_2G_scale)/1000;

			/* 角速度raw数据 */
			gyr_x_reg = (databuf[7] << 8) | databuf[6];
			gyr_y_reg = (databuf[9] << 8) | databuf[8];
			gyr_z_reg = (databuf[11] << 8) | databuf[10];

			/* 补码转原码 */
			dec_gyr_x = complementToOriginal(gyr_x_reg);
			dec_gyr_y = complementToOriginal(gyr_y_reg);
			dec_gyr_z = complementToOriginal(gyr_z_reg);
			
			/* 灵敏度转换 */
			gyr_x = ((float)dec_gyr_x * GYR_FSG_245_scale)/1000;
			gyr_y = ((float)dec_gyr_y * GYR_FSG_245_scale)/1000;
			gyr_z = ((float)dec_gyr_z * GYR_FSG_245_scale)/1000;

			/* 温度数据 */
			temp_reg = (databuf[13] << 8) | databuf[12];
			dec_temp = complementToOriginal(temp_reg);

			temp = ((float)dec_temp / 256.0) + 25.0;

			/* 姿态解算 */
			IMUupdate(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z);
			printf("Pitch = %f,\t\t\t Roll = %f\t\t\t Yaw = %f\r\n", Pitch, Roll, Yaw);
		}	
		//usleep(10000);	/* 50ms */
	}

	close(fd);	/* 关闭文件 */	
	return 0;
}

/* 补码转换为原码 */
int complementToOriginal(int complement)
{
	// 检查符号位
    if (complement & 0x8000) { // 如果符号位为1，即负数
        complement = complement - 0x10000; // 转换回原码
    }
    return complement;
}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float Roll_a;

	/* 将加速度计转化为单位向量 */
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	/* */
	/* 四元数表示三轴的重力分量 */
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	/* 求 “四元数所求的重力分量” 与 “加速度计测量值的误差 ” */
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	/* 修正陀螺仪测量值 */
	exInt = exInt + ex*Ki;
	gx = gx + Kp*ex + exInt;

	eyInt = eyInt + ey*Ki;
	gy = gy + Kp*ey + eyInt;

	ezInt = ezInt + ez*Ki;
	gz = gz + Kp*ez + ezInt; 

	/* 一阶龙格-库塔法解微分方程 */
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

	// 正常化四元数
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	/* 绕zyx顺序旋转 */
	Roll_zyx = (180.0f/pi)*asin(-2*(q1*q3-q0*q2)); 										// 绕y轴旋转	
	Pitch_zyx = (180.0f/pi)*atan2(2*(q0*q1+q2*q3),q0*q0-q1*q1-q2*q2+q3*q3);				// 绕x轴旋转
	Yaw_zyx = (180.0f/pi)*atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);				// 偏航角

	/* 绕zxy顺序旋转 */
	Pitch_zxy = (180.0f/pi)*asin(2*(q0*q1+q2*q3)); 										// 绕x轴旋转 俯仰角
	Roll_zxy = (180.0f/pi)*atan2(-2*(q0*q2 - q1*q3),q0*q0-q1*q1-q2*q2+q3*q3);			// 绕y轴旋转 横滚角
	Yaw_zxy = (180.0f/pi)*atan2(-q0*q0+q1*q1-q2*q2+q3*q3,2*(q1*q2-q0*q3));				// 偏航角

	Pitch = -Pitch_zxy;
	Roll = Roll_zxy + 180.0;
	Yaw = Yaw_zxy;

#if 0 
	if (Roll_zyx > -85 && Roll_zyx < 85 && Pitch_zxy > -85 && Pitch_zxy < 85) {
		Pitch = Pitch_zyx;
		Roll = Roll_zxy;
	} else if (Pitch_zxy < -85 || Pitch_zxy > 85) {
		Pitch = Pitch_zyx;
		Roll = Roll_zyx;
	} else if (Roll_zyx < -85 || Roll_zyx > 85) {
		Pitch = Pitch_zxy;
		Roll = Roll_zxy;
	}
#endif 
}
