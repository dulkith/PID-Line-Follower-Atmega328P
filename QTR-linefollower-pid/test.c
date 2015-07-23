#include <pololu/PololuQTRSensors.h>
#include <pololu/OrangutanDigital.h>
#include <pololu/OrangutanPushbuttons.h>
#include <pololu/OrangutanTime.h>
#include <pololu/OrangutanMotors.h>

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize()
{
	unsigned char qtr_rc_pins[] = {0,1,2,4,7,8,9,10};
	unsigned int counter; // used as a simple timer
	//unsigned int sensors[8]; // an array to hold sensor values

	// This must be called at the beginning of 3pi code, to set up the
	// sensors.  We use a value of 2000 for the timeout, which
	// corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
	qtr_rc_init(qtr_rc_pins, 8, 2000, 17);
										//*************************************************
	// Play welcome music and display a message
	//	*********
	// Blink RED LED  and wait for button press
	while(!button_is_pressed(BUTTON_B))
	{		
		set_digital_output(13,0xff);
		delay_ms(200);
	}

	// Always wait for the button to be released so that 3pi doesn't
	// start moving until your hand is away from it.
	wait_for_button_release(BUTTON_B);
	set_digital_output(13,LOW);
	delay_ms(1000);

	// Auto-calibration: turn right and left while calibrating the
	// sensors.
	
	for(counter=0;counter<80;counter++)
	{
		if(counter < 20 || counter >= 60)
			set_motors(90,-90);         //**************************************************
		else
			set_motors(-90,90);
		
		set_digital_output(14,HIGH);
		// This function records a set of sensor readings and keeps
		// track of the minimum and maximum values encountered.  The
		// IR_EMITTERS_ON argument means that the IR LEDs will be
		// turned on during the reading, which is usually what you
		// want.
		qtr_calibrate(QTR_EMITTERS_ON);   //**************************************************

		// Since our counter runs to 80, the total delay will be
		// 80*20 = 1700 ms.
		delay_ms(20);
	}
	set_digital_output(14,LOW);
	set_motors(0,0);

	while(!button_is_pressed(BUTTON_B))
	{
		set_digital_output(15,0xff);
		delay_ms(200);
	}

	// Always wait for the button to be released so that 3pi doesn't
	// start moving until your hand is away from it.
	wait_for_button_release(BUTTON_B);
	set_digital_output(15,LOW);
}
// Turns according to the parameter dir, which should be 'L', 'R', 'S'
// (straight), or 'B' (back).
void turn(char dir)
{
	switch(dir)
	{
		case 'L':
		// Turn left.
		set_motors(-90,90);
		delay_ms(200);
		break;
		case 'R':
		// Turn right.
		set_motors(90,-90);
		delay_ms(200);
		break;
		case 'T':
		// Don't do anything!
		break;
		case 'B':
		// Don't do anything!
		break;
	}
}
// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
char select_turn(unsigned char found_left, unsigned char found_right, unsigned char found_tee)
{
	// Make a decision about how to turn.  The following code
	// implements a left-hand-on-the-wall strategy, where we always
	// turn as far to the left as possible.
	if(found_left)
	return 'L';
	else if(found_right)
	return 'R';
	else if(found_tee)
	return 'T';
	else
	return 'B';
}
// This is the main function, where the code starts.  All C programs
// must have a main() function defined somewhere.
int main()
{
	unsigned int sensors[8]; // an array to hold sensor values
	unsigned int last_proportional=0;
	long integral=0;
	
	// set up the 3pi
	initialize();

	// This is the "main loop" - it will run forever.
	while(1)
	{   
		unsigned char found_left=0;
		unsigned char found_tee=0;
		unsigned char found_right=0;
		
		set_digital_output(16,HIGH);
		// Get the position of the line.  Note that we *must* provide
		// the "sensors" argument to read_line() here, even though we
		// are not interested in the individual sensor readings.
		unsigned int position = qtr_read_line(sensors,QTR_EMITTERS_ON);
		
		if(sensors[0] > 700 && sensors[1] > 700 && sensors[2] > 700)    //Finding Left 
		found_left = 1;
		if(sensors[7] > 700 && sensors[6] > 700 && sensors[5] > 700)	//Finding Right
		found_right = 1;
		if(sensors[7] > 700 && sensors[6] > 700 && sensors[5] > 700 && sensors[4] > 700 && sensors[3] > 700 && sensors[2] > 700 && sensors[1] > 700 && sensors[0] > 700)	//Finding Tee
		found_tee = 1;
		
		
		unsigned char dir = select_turn(found_left,found_right,found_tee);

		// Make the turn indicated by the path.
		turn(dir);
		
		position = qtr_read_line(sensors,QTR_EMITTERS_ON);
		
		// The "proportional" term should be 0 when we are on the line.
		int proportional = ((int)position) - 3500; //*****************************************

		// Compute the derivative (change) and integral (sum) of the
		// position.
		int derivative = proportional - last_proportional;
		integral += proportional;

		// Remember the last position.
		last_proportional = proportional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the right.  If it is a negative number, the robot will
		// turn to the left, and the magnitude of the number determines
		// the sharpness of the turn.
		int power_difference = proportional/20 + integral/10000 + derivative*3/2;

		// Compute the actual motor settings.  We never set either motor
		// to a negative value.
		const int max = 165;
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;

		if(power_difference < 0)
			set_motors(max+power_difference, max);
		else
			set_motors(max, max-power_difference);
	}

	// This part of the code is never reached.  A robot should
	// never reach the end of its program, or unpredictable behavior
	// will result as random code starts getting executed.  If you
	// really want to stop all actions at some point, set your motors
	// to 0,0 and run the following command to loop forever:
	//
	// while(1);
}

// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
