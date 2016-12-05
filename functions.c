void lift(int power) {
	motor[liftLeftTop] = motor[liftRightTop] = power;
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
	motor[leftBack] = motor[leftMiddle] = motor[leftFront] = motor[leftBackTop] = power;
}

void driveR(float power) {
	motor[rightBack] = motor[rightMiddle] = motor[rightFront] = motor[rightBackTop] = power;
}

int t = 15;
int ch3, ch1, ch2;

void arcadeDrive() {
	if (abs(vexRT[Ch3]) > t)
		ch3 = vexRT[Ch3];
	else
		ch3 = 0;

	if (abs(vexRT[Ch1]) > t)
		ch1 = (1/(pow(127,2)) * (pow(vexRT[Ch1], 3);
	else
		ch1 = 0;

	driveL(ch3 + ch1);
	driveR(ch3 - ch1);
}

void tankDrive() {
	if (abs(vexRT[Ch3]) > t)
		ch3 = vexRT[Ch3];
	else
		ch3 = 0;

	if (abs(vexRT[Ch2]) > t)
		ch2 = vexRT[Ch2];
	else
		ch2 = 0;

	driveL(ch3);
	driveR(ch2);
}

string mainBattery, backupBattery;

void lcd() {
	clearLCDLine(0);
	clearLCDLine(1);

	displayLCDString(0, 0, "Primary: ");
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
	displayNextLCDString(mainBattery);

	displayLCDString(1, 0, "Backup: ");
	sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');    //Build the value to be displayed
	displayNextLCDString(backupBattery);

	wait1Msec(100);
}
