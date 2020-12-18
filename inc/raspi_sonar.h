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

#ifdef __cplusplus
}
#endif
#endif
