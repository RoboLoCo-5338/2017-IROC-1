package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Shoot extends Command {

	public Shoot() {
		requires(Robot.shooter);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.shooter.shoot();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}
}
