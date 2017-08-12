package org.usfirst.frc.team5338.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

import org.usfirst.frc.team5338.robot.subsystems.DriveTrain;

public class Robot extends IterativeRobot {

    public static final OI oi = new OI();
    public static final DriveTrain drivetrain = new DriveTrain();

    @Override
    public void robotInit() {
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