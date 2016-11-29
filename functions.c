void lift(int power) {
	motor[liftLeftTop] = motor[liftLeftBottom] = motor[liftRightTop] = motor[liftRightBottom] = power;
}

void liftControl() {
	if (vexRT[Btn6U])
		lift(127);
	else if (vexRT[Btn6D])
		lift(-127);
	else
		lift(0);
}

void driveL(float power) {
	motor[leftBack] = motor[leftMiddle] = motor[leftFront] = power;
}

void driveR(float power) {
	motor[rightBack] = motor[rightMiddle] = motor[rightFront] = power;
}

int t = 15;
int ch3, ch1;

void arcadeDrive() {
	if (abs(vexRT[Ch3]) > t)
		ch3 = vexRT[Ch3];
	else
		ch3 = 0;

	if (abs(vexRT[Ch1]) > t)
		ch1 = vexRT[Ch1];
	else
		ch1 = 0;

	driveL(ch3 + ch1);
	driveR(ch3 - ch1);
}
