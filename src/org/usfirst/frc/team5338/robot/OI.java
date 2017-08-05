package org.usfirst.frc.team5338.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private final Joystick joyL = new Joystick(0);
    private final Joystick joyR = new Joystick(1);

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

    private double joystickDeadZone(double value) {
	if (value > 0.05) {
	    return (value - 0.05) / 0.95;
	} else if (value < -0.05) {
	    return (value + 0.05) / 0.95;
	}
	return value;
    }

    public double getLeftX() {
	return joystickDeadZone(joyL.getRawAxis(1));
    }
    
    public double getLeftY() {
    return joystickDeadZone(joyL.getRawAxis(2));
    }

    public double getRightX() {
	return joystickDeadZone(joyR.getRawAxis(1));
    }
    
    public double getRightY() {
    return joystickDeadZone(joyR.getRawAxis(1));
    }
}
