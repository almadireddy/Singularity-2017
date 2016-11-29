void lift(int power) {
	motor[topRight] = motor[topLeft] = motor[bottomRight] = motor[bottomLeft] = power;
}

void liftControl() {
	if (vexRT[Btn8U])
		lift(127);
	else if (vexRT[Btn8D])
		lift(-127);
	else
		lift(0);
}
