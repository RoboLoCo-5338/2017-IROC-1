package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.LiftRobot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Winch extends Subsystem {
	private final CANTalon LIFT = new CANTalon(7);

	public Winch() {
		super();
		LIFT.enable();
	}

	public void initDefaultCommand() {
		setDefaultCommand(new LiftRobot());
	}

	public void liftRobot(OI oi) {
		if (oi.get(OI.Button.CLIMB)) {
			LIFT.set(-0.99);
			return;
		} else {
			stopLift();
		}
	}

	public void stopLift() {
		LIFT.set(0.0);
	}
}
