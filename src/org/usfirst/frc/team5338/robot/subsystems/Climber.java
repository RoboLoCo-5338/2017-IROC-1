package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.Climb;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {
	private final VictorSP LIFT = new VictorSP(0);

	public Climber() {
		super();
	}

	public void initDefaultCommand() {
		setDefaultCommand(new Climb());
	}

	public void liftRobot(OI oi) {
		if (oi.get(OI.Button.SLOW_CLIMB)) {
			LIFT.set(0.333);
		}
		else if (oi.get(OI.Button.FAST_CLIMB)) {
			LIFT.set(1.0);
		} else {
			stopLift();
		}
	}

	public void stopLift() {
		LIFT.set(0.0);
	}
}
