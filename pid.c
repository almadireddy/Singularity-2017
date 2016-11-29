// Lift PID //
float liftKp = 0.3;
float liftKi = 0.1;
float liftKd = 0.4;
float liftTarget, liftError, liftLastError, liftIntegral, liftDerivative, liftOutput;
float liftIntegralCap = 50;
float dt = 25;


task liftPID() {
	liftLastError = 0;
	liftIntegral  = 0;

	while (true) {
		liftError = nMotorEncoder[liftRightTop] - nMotorEncoder[liftLeftTop];
		liftIntegral += dt * (liftError + liftLastError)/2;

		if(liftError == 0)
			liftIntegral = 0;
		if(fabs(liftIntegral) > liftIntegralCap)
			liftIntegral = liftIntegralCap;

		liftDerivative = (liftError - liftLastError)/dt;
		liftOutput = liftKp*liftError + liftKi*liftIntegral + liftKd*liftDerivative;
		liftLastError = liftError;
		wait1Msec(dt);
	}
}
// End Lift PID //

// Drive PID //
float driveKp = 0.3;
float driveKi = 0.1;
float driveKd = 0.4;
float driveCurrentValue;
float driveTarget, driveError, driveLastError, driveIntegral, driveDerivative, driveOutput;
float driveIntegralCap = 50;
float ticksPerInch = 360/(4 * PI);
float driveMax = 80;
float driveMin = -80;
float secondary, constant, difference;

task drivePID() {
	driveLastError  = 0;
	driveIntegral   = 0;

	while(true) {
		if (abs(SensorValue[leftEncoder]) > abs(SensorValue[rightEncoder])) {
			driveCurrentValue = SensorValue[rightEncoder];
			secondary = -SensorValue[leftEncoder];
			difference = abs(secondary) - abs(driveCurrentValue);
			driveL(-driveOutput + constant*difference);
			driveR(-driveOutput);
		}
		else {
			driveCurrentValue = SensorValue[leftEncoder];
			secondary = SensorValue[rightEncoder];
			difference = abs(secondary) - abs(driveCurrentValue);
			driveL(-driveOutput);
			driveR(-driveOutput + constant*difference);
		}

		driveError = driveCurrentValue - driveTarget;

		if( driveKi != 0 ) {
			if( abs(driveError) < driveIntegralCap)
				driveIntegral = driveIntegral + driveError;
			else
				driveIntegral = 0;
		}
		else
			driveIntegral = 0;

		driveDerivative = driveError - driveLastError;
		driveLastError  = driveError;

		driveOutput = (driveKp * driveError) + (driveKi * driveIntegral) + (driveKd * driveDerivative);

		if ( driveOutput > driveMax )
			driveOutput = driveMax;
		if ( driveOutput < driveMin )
			driveOutput = driveMin;

    wait1Msec(dt);
	}
}

void go(float inches) {
	SensorValue[leftEncoder] = SensorValue[rightEncoder] = 0;

	driveTarget = inches * ticksPerInch;
}
