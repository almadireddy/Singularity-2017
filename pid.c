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
float rev = 360;
float ticksPerInch = rev/(4 * PI);
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
	startTask(drivePID);
	SensorValue[leftEncoder] = SensorValue[rightEncoder] = 0;

	driveTarget = inches * ticksPerInch;
}
// End Drive PID //

// Gyro PID //
float currentValueGyro;
float currentValueGyro2;
float targetGyro = 0;
float errorGyro;
float speedGyro;
float kpGyro = 0.075;
float kiGyro = 0.07;
float kdGyro = 0.12;
float integralGyro;
float lastErrorGyro;
float derivativeGyro;

float estimate = 1; //current estimate
float Gestimate = 0.001; //current estimate
float G2estimate = 0.001; //current estimate
float G3estimate = 0.001; //current estimate
float gyroAvg;

task gyroDrift() {
	int gyroError = 0;
	float lastGyro;
	while(true) {
		if ( abs(lastGyro - SensorValue[gyro]) < 2)
			gyroError += lastGyro - SensorValue[gyro];

		lastGyro = SensorValue[gyro];
		currentValueGyro = SensorValue[gyro] + gyroError;

		wait1Msec(100);
	}
}

//task gyroTurn() {
//	while(true) {
//		errorGyro = targetGyro - currentValueGyro;
//		derivativeGyro = errorGyro - lastErrorGyro;
//		integralGyro = integralGyro + lastErrorGyro;
//		lastErrorGyro = errorGyro;
//		if(abs(integralGyro) > 380)
//		{
//			integralGyro = 380;
//		}
//		if(errorGyro == 0)
//		{
//			integralGyro = 0;
//		}

//		speedGyro = kpGyro * errorGyro + kiGyro * integralGyro + kdGyro * derivativeGyro;

//		if(speedGyro > driveMax)
//			speedGyro = driveMax;
//		if(speedGyro < driveMin)
//			speedGyro = driveMin;

//		driveR(speedGyro);
//		driveL(-speedGyro);

//		wait1Msec(25);
//	}
//}

task gyroTurn() {
	while(true) {
		errorGyro = targetGyro - currentValueGyro;
		derivativeGyro = errorGyro - lastErrorGyro;
		integralGyro = integralGyro + lastErrorGyro;
		lastErrorGyro = errorGyro;
		if(abs(integralGyro) > 380)
		{
			integralGyro = 380;
		}
		if(errorGyro == 0)
		{
			integralGyro = 0;
		}

		speedGyro = kpGyro * errorGyro + kiGyro * integralGyro + kdGyro * derivativeGyro;

		driveR(-speedGyro);
		driveL(speedGyro);

		wait1Msec(25);
	}
}

task kFilter() {
	float kG; //kalman gain
	float Eest = 10; // error in esitmate
	float Emea = 20; //error in measurement

	float lastEstimate; //last estimate
	float measurement;

	float GkG; //kalman gain
	float GEest = 10; // error in esitmate
	float GEmea = 20; //error in measurement

	float GlastEstimate; //last estimate
	float Gmeasurement;

	float G2kG; //kalman gain
	float G2Eest = 10; // error in esitmate
	float G2Emea = 20; //error in measurement

	float G2lastEstimate; //last estimate
	float G2measurement;

	float G3kG; //kalman gain
	float G3Eest = 10; // error in esitmate
	float G3Emea = 20; //error in measurement

	float G3lastEstimate; //last estimate
	float G3measurement;

	while(true) {
		lastEstimate = estimate;
		measurement = SensorValue[gyro];

		kG = Eest / (Eest + Emea);

		estimate = lastEstimate + kG * (measurement - lastEstimate);
		Eest = (1 - kG) * (lastEstimate);

		GlastEstimate = Gestimate;
		Gmeasurement = currentValueGyro;

		GkG = GEest / (GEest + GEmea);

		Gestimate = GlastEstimate + GkG * (Gmeasurement - GlastEstimate);
		GEest = (1 - GkG) * (GlastEstimate);

		G2lastEstimate = G2estimate;
		G2measurement = currentValueGyro2;

		G2kG = G2Eest / (G2Eest + G2Emea);

		G2estimate = G2lastEstimate + G2kG * (G2measurement - G2lastEstimate);
		G2Eest = (1 - G2kG) * (G2lastEstimate);

		gyroAvg = (Gestimate + G2estimate) / 2;

		G3lastEstimate = G3estimate;
		G3measurement = gyroAvg;

		G3kG = G3Eest / (G3Eest + G3Emea);

		G3estimate = G3lastEstimate + G3kG * (G3measurement - G3lastEstimate);
		G3Eest = (1 - G3kG) * (G3lastEstimate);
		wait1Msec(25);
	}
}

void startGyroTasks() {
	//startTask(kFilter);
	startTask(gyroTurn);
	startTask(gyroDrift);
}

void stopGyroTasks() {
	//stopTask(kFilter);
	stopTask(gyroTurn);
	stopTask(gyroDrift);
}

void turn(float degrees) {
	//startGyroTasks();
	targetGyro = degrees*10;
}
