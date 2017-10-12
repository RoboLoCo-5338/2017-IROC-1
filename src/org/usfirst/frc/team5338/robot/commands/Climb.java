package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Climb extends Command {
    public Climb() {
	requires(Robot.winch);
    }

    protected void execute() {
	Robot.winch.liftRobot(Robot.oi);
    }

    protected boolean isFinished() {
	return false;
    }

    protected void end() {
	Robot.winch.stopLift();
    }
}