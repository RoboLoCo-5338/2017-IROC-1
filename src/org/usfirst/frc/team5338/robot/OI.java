package org.usfirst.frc.team5338.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
	// Creates the two Joystick objects.
	private final Joystick joyLeft = new Joystick(0);
	private final Joystick joyRight = new Joystick(1);

	// Button enum definition for all usable buttons.
	public enum Button {
		BUTTON1, BUTTON2, BUTTON3, BUTTON4
	}

	// OI object constructor.
	public OI() {
	}

	// Gets boolean button status using a switch statement based on the button
	// argument.
	public boolean get(Button button) {
		switch (button) {
		case BUTTON1: // Returns left joystick trigger status
			return joyLeft.getRawButton(1);
		case BUTTON2: // Returns right joystick trigger status
			return joyRight.getRawButton(1);
		case BUTTON3: // Returns left joystick side button status
			return joyLeft.getRawButton(2);
		case BUTTON4: // Returns right joystick side button status
			return joyRight.getRawButton(2);
		default:
			return false;
		}
	}

	// Gets a corrected double value after adjusting for a deadzone around 0.
	private double deadZoneCorrection(double value) {
		if (value > 0.06) { // Values are scaled from 0.04 to 1 to 0 to 1
			return (value - 0.06) / 0.94;
		} else if (value < -0.06) { // Values are scaled from -0.04 to -1 to 0 to -1
			return (value + 0.06) / 0.94;
		} else { // Returns 0
			return 0.0;
		}
	}

	// Gets double joystick values based on character argument
	public double getLeft(Character string) {
		switch (string) {
		case 'X': // Gets deadzone corrected x-axis position
			return deadZoneCorrection(joyLeft.getRawAxis(0));
		case 'Y': // Gets deadzone corrected y-axis position
			return deadZoneCorrection(joyLeft.getRawAxis(1));
		case 'Z': // Gets deadzone corrected z-axis (rotation) position
			return deadZoneCorrection(joyLeft.getRawAxis(2));
		case 'T': // Gets throttle position
			return (joyLeft.getRawAxis(3) - 1)/-2;
		case 'M': // Gets deadzone corrected magnitude away from origin
			return deadZoneCorrection(joyLeft.getMagnitude());
		case 'A': // Gets angle of joystick in radians if greater than a deadzone
			if(deadZoneCorrection(joyLeft.getMagnitude()) > 0.06)
			{
			return joyLeft.getDirectionDegrees();
			}
			else
			{
			return -200.0;
			}
		default:
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
			return (joyRight.getRawAxis(3) - 1)/-2;
		case 'M': // Gets deadzone corrected magnitude away from origin
			return deadZoneCorrection(joyRight.getMagnitude());
		case 'A': // Gets angle of joystick in radians
			if(deadZoneCorrection(joyRight.getMagnitude()) > 0.06)
			{
			return joyRight.getDirectionDegrees();
			}
			else
			{
			return -200.0;
			}
		default:
			return 0.0;
		}
	}
	
}