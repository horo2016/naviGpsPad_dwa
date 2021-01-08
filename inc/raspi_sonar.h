#ifndef RASPI_SONAR_H
#define RASPI_SONAR_H
#ifdef __cplusplus
extern "C" {
#endif

typedef  struct{
	float distance;
	
}sonar_dis;

extern sonar_dis raspi_sonars[3];

extern  void *getUltrasonicThread(void *arg);

extern void obstacleAvoidance();


uint8_t MIN_RANGE_OBSTACLE ; //Between 0 and 5 cm is the blind zone of the sensor.
uint8_t MAX_RANGE_OBSTACLE ;

#ifdef __cplusplus
}
#endif
#endif
