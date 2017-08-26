package org.usfirst.frc.team5338.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
	private final static Joystick joystickLeft = new Joystick(0);
	private final static Joystick joystickRight = new Joystick(1);

	public enum Button {
		TEST1, TEST2
	}

	public OI() {
	}

	public static Joystick getJoystick(int n) {
		if (n == 0)
			return joystickLeft;
		else if (n == 1)
			return joystickRight;
		else
			return null;
	}

	public boolean get(Button button) {
		switch (button) {
		case TEST1:
			return joystickLeft.getRawButton(1);
		case TEST2:
			return joystickRight.getRawButton(1);
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
			return joystickDeadZone(joystickLeft.getRawAxis(1));
		case "Y":
			return joystickDeadZone(joystickLeft.getRawAxis(2));
		case "Z":
			return joystickDeadZone(joystickLeft.getRawAxis(3));
		default:
			return 0.0;
		}
	}

	public double getRight(String input) {
		switch (input) {
		case "X":
			return joystickDeadZone(joystickRight.getRawAxis(1));
		case "Y":
			return joystickDeadZone(joystickRight.getRawAxis(2));
		case "Z":
			return joystickDeadZone(joystickRight.getRawAxis(3));
		default:
			return 0.0;
		}
	}

	public double getDirectionDegrees(int n) {
		switch (n) {
		case 0:
			return joystickLeft.getDirectionDegrees();
		case 1:
			return joystickRight.getDirectionDegrees();
		default:
			return 0.0;
		}
	}
}