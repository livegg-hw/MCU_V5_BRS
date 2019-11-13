typedef enum
{
		ACCELEROMETER_AXIS_X,
		ACCELEROMETER_AXIS_Y,
		ACCELEROMETER_AXIS_Z,
} ACCELEROMETER_AXIS;

void    LIS3DSHTR_initialization(void);
//int16_t LIS3DSHTR_get_accelerometerc_sample(ACCELEROMETER_AXIS accelerometer_axis);
uint16_t lis3dshtr_get_accelerometer_axis_sample(ACCELEROMETER_AXIS accelerometer_axis);
void lis3dshtr_get_accelerometer_sample(SAMPLE *sample_pointer);

int16_t LIS3DSHTR_get_accelerometer_axis_x();
int16_t LIS3DSHTR_get_accelerometer_axis_y();
int16_t LIS3DSHTR_get_accelerometer_axis_z();
int8_t  LIS3DSHTR_get_temperature();
void get_lis3dshtr_sample(void);
