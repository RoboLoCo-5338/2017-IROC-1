package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SwerveDriveWithJoysticks extends Command {
	public SwerveDriveWithJoysticks() {
		requires(Robot.drivetrain);
	}

	@Override
	protected void execute() {
		Robot.drivetrain.drive(Robot.oi);
	}
	//Saarthak

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		Robot.drivetrain.drive(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	}
}