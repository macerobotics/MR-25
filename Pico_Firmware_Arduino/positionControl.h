#ifndef POSITION_CONTROL_h
#define POSITION_CONTROL_h

// Nombre de ticks pour 1 tour de roue
#define ENCODER_RESOLUTION 4200


void positionControl_manage();

int positionRobotX(void);
int positionRobotY(void);

// vitesse des roues
int speedwheelRight(void);
int speedwheelLeft(void);


#endif
// End of file
