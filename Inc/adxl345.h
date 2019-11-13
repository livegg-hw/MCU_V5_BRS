#ifndef __ADXL345_H
#define __ADXL345_H

void adxl345_initialization(void);
void ADXL345_clear_interrupt(void);
uint16_t adxl345_get_accelerometer_axis_sample(AXIS axis);
void adxl345_get_accelerometer_sample(SAMPLE *sample_pointer);
void adxl345_get_accelerometer_accurate_sample(ACCURATE_SAMPLE *accurate_sample);
void adxl345_get_accelerometer_sample_FeedbackFiltered(ACCURATE_SAMPLE *accurate_sample , const float c1, const float maxDiff, const uint32_t minMeasurements, const uint32_t maxMeasurements);
void adxl345_get_accelerometer_to_ground_matrix(ACCURATE_SAMPLE accurate_sample, float* R);
uint8_t adxl345_get_device_id();
void get_adxl345_sample(void);

uint8_t adxl345_get_register(uint8_t adxl345_register);

#endif // __ADXL345_H
