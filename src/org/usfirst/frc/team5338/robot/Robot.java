package org.usfirst.frc.team5338.robot;

import org.usfirst.frc.team5338.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends IterativeRobot {
	public static final OI oi = new OI();
	public static final DriveTrain drivetrain = new DriveTrain();

	@Override
	public void robotInit() {
		DriveTrain.initSensors();
	}
	

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
}