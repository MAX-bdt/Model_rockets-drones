/*
Functions to send a pulse on remaining analog pins to measure loop/function speed using an oscilloscope
*/

// delay in us
void Pulse_PIN_A6(uint16_t del) {
	digitalWrite(A6, 0);
	digitalWrite(A6, 1);
	delayMicroseconds(del);
	digitalWrite(A6, 0);

}

// delay in us
void Pulse_PIN_A7(uint16_t del) {
	digitalWrite(A7, 0);
	digitalWrite(A7, 1);
	delayMicroseconds(del);
	digitalWrite(A7, 0);

}
