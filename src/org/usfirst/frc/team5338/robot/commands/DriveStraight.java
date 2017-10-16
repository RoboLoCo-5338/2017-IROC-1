package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;
import org.usfirst.frc.team5338.robot.subsystems.Drivetrain;
import org.usfirst.frc.team5338.robot.subsystems.Drivetrain.Vector;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command {

    public DriveStraight(int time) {
        requires(Robot.drivetrain);
        setTimeout(time);
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		Vector straight = Robot.drivetrain.new Vector(0, 0.5);
    		Robot.drivetrain.drive(straight, straight, straight, straight);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }
}
