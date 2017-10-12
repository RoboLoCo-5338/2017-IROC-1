package org.usfirst.frc.team5338.robot;

import org.usfirst.frc.team5338.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends IterativeRobot {
	// Creates the OI, DriveTrain, Shooter, and Winch objects.
	public static final OI oi = new OI();
	public static final DriveTrain drivetrain = new DriveTrain();
	public static final Shooter shooter = new Shooter();
	public static final Winch winch = new Winch();

	// Creates the autonomous object.
	private static final Command autonomous = new Autonomous();

	// Robot object constructor.
	public void robotInit() {
	}

	// Called once to start the autonomous command.
	public void autonomousInit() {
		autonomous.start();
	}

	// Periodically called to run the autonomous command.
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	// Called once to end the autonomous command.
	public void teleopInit() {
		try {
			autonomous.cancel();
		} catch (Exception e) {
		}
	}

	// Periodically called to run default commands in teleop.
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
}