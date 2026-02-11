// copyleft 

#ifndef CONTROL_ROBOT_h
#define CONTROL_ROBOT_h

bool IRQ_controlRobot(struct repeating_timer *t);

void controlRobot_manage(void);

void controlRobot_enable(bool activation);

bool controlRobot_state(void);

void controlRobot_WriteCommandeDistance(float consigne);

void controlRobot_WriteCommandeOrientation(float consigne);

#endif
// End of file
