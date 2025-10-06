#ifndef ROBOT_h
#define ROBOT_h

#define pin_BUZZER 19

void init_robot();
void forwardRobot(int val);
void forwardmm(int distance);
void turnAngle(int angle);
void backRobot(int val);
void stopRobot();
void turnRight(int vitesse);
void turnLeft(int vitesse);
void motorRight(int dir, int pwm);
void motorLeft(int dir, int pwm);

void ledRgb(bool r, bool g, bool b);

int encoderLeft(void);
int encoderRight(void);
void encoderReset(void);



#endif
// End of file
