package org.usfirst.frc.team5338.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
	// Creates the two Joystick objects.
	private final Joystick joyLeft = new Joystick(0);
	private final Joystick joyRight = new Joystick(1);

	// Button enum definition for all usable buttons on both controllers.
	public enum Button {
		NO_ROTATION, NO_TRANSLATION, SHOOT, CLIMB, RESET_YAW_1, RESET_YAW_2
	}

	// OI object constructor.
	public OI() {
	}

	// Gets boolean button status using a switch statement based on the button
	// argument.
	public boolean get(Button button) {
		switch (button) {
		case NO_ROTATION: // Returns left joystick trigger status
			return joyLeft.getRawButton(1);
		case CLIMB: // Returns right joystick trigger status
			return joyLeft.getRawButton(2);
		case NO_TRANSLATION: // Returns left joystick side button status
			return joyRight.getRawButton(1);
		case SHOOT: // Returns right joystick side button status
			return joyRight.getRawButton(2);
		case RESET_YAW_1: // Returns left joystick button 7 status
			return joyLeft.getRawButton(7);
		case RESET_YAW_2: // Returns left joystick button 8 status
			return joyLeft.getRawButton(8);
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
	public double getLeft(Character input) {
		switch (input) {
		case 'X': // Gets deadzone corrected x-axis position
			return deadZoneCorrection(joyLeft.getRawAxis(0));
		case 'Y': // Gets deadzone corrected y-axis position
			return deadZoneCorrection(joyLeft.getRawAxis(1));
		case 'Z': // Gets deadzone corrected z-axis (rotation) position
			return deadZoneCorrection(joyLeft.getRawAxis(2));
		case 'T': // Gets throttle position
			return (joyLeft.getRawAxis(3) - 1) / -2;
		case 'M': // Gets deadzone corrected magnitude away from origin
			return deadZoneCorrection(joyLeft.getMagnitude());
		case 'A': // Gets angle of joystick in degrees if magnitude is greater than 0.1
			if (joyLeft.getMagnitude() > 0.1) {
				return joyLeft.getDirectionDegrees();
			} else { // Returns 999 is magnitude is less than 0.1
				return 999;
			}
		default: // Returns 0.0 is argument is unknown
			return 0.0;
		}
	}

	// Gets double joystick values based on character argument
	public double getRight(Character input) {
		switch (input) {
		case 'X': // Gets deadzone corrected x-axis position
			return deadZoneCorrection(joyRight.getRawAxis(0));
		case 'Y': // Gets deadzone corrected y-axis position
			return deadZoneCorrection(joyRight.getRawAxis(1));
		case 'Z': // Gets deadzone corrected z-axis (rotation) position
			return deadZoneCorrection(joyRight.getRawAxis(2));
		case 'T': // Gets throttle position
			return (joyRight.getRawAxis(3) - 1) / -2;
		case 'M': // Gets deadzone corrected magnitude away from origin
			return deadZoneCorrection(joyLeft.getMagnitude());
		case 'A': // Gets angle of joystick in degrees if magnitude is greater than 0.1
			if (joyRight.getMagnitude() > 0.1) {
				return joyRight.getDirectionDegrees();
			} else { // Returns 999 is magnitude is less than 0.1
				return 999;
			}
		default: // Returns 0.0 is argument is unknown
			return 0.0;
		}
	}

}