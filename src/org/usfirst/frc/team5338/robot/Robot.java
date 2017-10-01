package org.usfirst.frc.team5338.robot;

import org.usfirst.frc.team5338.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends IterativeRobot {
	// Creates the OI and DriveTrain objects.
	public static final OI oi = new OI();
	public static final DriveTrain drivetrain = new DriveTrain();

	// Robot object constructor.
	public void robotInit() {
	}

	public void autonomousInit() {
	}

	// Periodically called to run the autonomous command.
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
	}

	// Periodically called to run default commands in teleop.
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
}