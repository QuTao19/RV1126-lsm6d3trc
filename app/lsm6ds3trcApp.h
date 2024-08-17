#ifndef LSM6DS3TRCAPP_H
#define LSM6DS3TRCAPP_H


#define GYR_FSG_245_scale 8.750f
#define GYR_FSG_500_scale 17.50f
#define GYR_FSG_1000_scale 35.0f
#define GYR_FSG_2000_scale 70.0f

#define ACC_FSXL_2G_scale 0.061f
#define ACC_FSXL_4G_scale 0.122f
#define ACC_FSXL_8G_scale 0.244f
#define ACC_FSXL_16G_scale 0.488f

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
int complementToOriginal(int complement);

#endif