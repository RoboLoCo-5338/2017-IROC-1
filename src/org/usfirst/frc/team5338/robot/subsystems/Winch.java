package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.Climb;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Winch extends Subsystem {
	private final VictorSP LIFT = new VictorSP(1);

	public Winch() {
		super();
	}

	public void initDefaultCommand() {
		setDefaultCommand(new Climb());
	}

	public void liftRobot(OI oi) {
		if (oi.get(OI.Button.CLIMB)) {
			LIFT.set(0.99);
			return;
		} else {
			stopLift();
		}
	}

	public void stopLift() {
		LIFT.set(0.0);
	}
}
