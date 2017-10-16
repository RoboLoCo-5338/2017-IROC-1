package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Shoot extends Command {
	public Shoot() {
		requires(Robot.shooter);
	}

	protected void execute() {
		Robot.shooter.shoot();
	}

	protected boolean isFinished() {
		return false;
	}
}
