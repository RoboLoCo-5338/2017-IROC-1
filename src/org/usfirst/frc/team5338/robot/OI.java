package org.usfirst.frc.team5338.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
	private final Joystick joyL = new Joystick(0);
	private final Joystick joyR = new Joystick(1);

	public enum Button {
		TEST1, TEST2
	}

	public OI() {
	}

	public Joystick getJoystick(int n) {
		if (n == 0)
			return joyL;
		else if (n == 1)
			return joyR;
		else
			return null;
	}

	public boolean get(Button button) {
		switch (button) {
		case TEST1:
			return joyR.getRawButton(1);
		case TEST2:
			return joyL.getRawButton(1);
		default:
			return false;
		}
	}

	private double joystickDeadZone(double value) {
		if (value > 0.05) {
			return (value - 0.05) / 0.95;
		} else if (value < -0.05) {
			return (value + 0.05) / 0.95;
		}
		return value;
	}

	public double getLeft(String input) {
		switch (input) {
		case "X":
			return joystickDeadZone(joyL.getRawAxis(1));
		case "Y":
			return joystickDeadZone(joyL.getRawAxis(2));
		case "Z":
			return joystickDeadZone(joyL.getRawAxis(3));
		default:
			return 0.0;
		}
	}

	public double getRight(String input) {
		switch (input) {
		case "X":
			return joystickDeadZone(joyR.getRawAxis(1));
		case "Y":
			return joystickDeadZone(joyR.getRawAxis(2));
		case "Z":
			return joystickDeadZone(joyR.getRawAxis(3));
		default:
			return 0.0;
		}
	}
}