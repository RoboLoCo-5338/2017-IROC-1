package org.usfirst.frc.team5338.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
	// Creates the Joystick object.
	private final Joystick joystick = new Joystick(0);

	// Button enum definition for all usable buttons on both controllers.
	public enum Button {
		NO_ROTATION, NO_TRANSLATION, SLOW_CLIMB, FAST_CLIMB, SHOOT, RESET_YAW_1, RESET_YAW_2, SHOOT_FARTHER, SHOOT_CLOSER
	}

	// OI object constructor.
	public OI() {
	}

	// Gets boolean button status using a switch statement based on the button
	// argument.
	public boolean get(Button button) {
		switch (button) {
		case NO_ROTATION: // Returns joystick trigger status
			return joystick.getRawButton(1);
		case NO_TRANSLATION: // Returns joystick side button status
			return joystick.getRawButton(2);
		case SLOW_CLIMB: // Returns joystick button 3 status
			return joystick.getRawButton(3);
		case FAST_CLIMB: // Returns joystick button 5 status
			return joystick.getRawButton(5);
		case SHOOT: // Returns joystick button 4 status
			return joystick.getRawButton(4);
		case RESET_YAW_1: // Returns joystick button 7 status
			return joystick.getRawButton(7);
		case RESET_YAW_2: // Returns joystick button 8 status
			return joystick.getRawButton(8);
		case SHOOT_CLOSER: // Returns joystick button 9 status
			return joystick.getRawButton(9);
		case SHOOT_FARTHER: // Returns joystick button 10 status
			return joystick.getRawButton(10);
		default:
			return false;
		}
	}

	// Gets a corrected double value after adjusting for a deadzone around 0.
	private double deadZoneCorrection(double value) {
		if (value > 0.1) { // Values are scaled from 0.1 to 1 to 0 to 1
			return (value - 0.1) / 0.90;
		} else if (value < -0.1) { // Values are scaled from -0.1 to -1 to 0 to -1
			return (value + 0.1) / 0.90;
		} else { // Returns 0 from -0.1 to 0.1
			return 0.0;
		}
	}

	// Gets double joystick values based on character argument
	public double get(Character input) {
		switch (input) {
		case 'X': // Gets deadzone corrected x-axis position
			return deadZoneCorrection(joystick.getRawAxis(0));
		case 'Y': // Gets deadzone corrected y-axis position
			return deadZoneCorrection(joystick.getRawAxis(1));
		case 'Z': // Gets deadzone corrected z-axis (rotation) position
			return deadZoneCorrection(joystick.getRawAxis(2));
		case 'T': // Gets throttle position
			return (joystick.getRawAxis(3) - 1) / -2;
		case 'M': // Gets deadzone corrected magnitude away from origin
			return deadZoneCorrection(joystick.getMagnitude());
		case 'A': // Gets angle of joystick in degrees if magnitude is greater than 0.1
			if (joystick.getMagnitude() > 0.1) {
				return joystick.getDirectionDegrees();
			} else { // Returns 999 is magnitude is less than 0.1
				return 999;
			}
		default: // Returns 0.0 is argument is unknown
			return 0.0;
		}
	}
}