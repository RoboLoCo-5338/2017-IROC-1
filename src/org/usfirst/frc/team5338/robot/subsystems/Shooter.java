package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.commands.Shoot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
	// Creates the four CANTalon motor controller objects.
	private final CANTalon Test2 = new CANTalon(50, 1);
	private final CANTalon Test1 = new CANTalon(51, 1);
	private final VictorSP Test3 = new VictorSP(9);

	// DriveTrain object constructor which reverses output of backwards motors.
	public Shooter() {
		super();
		Test2.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		Test1.changeControlMode(TalonControlMode.Follower);
		Test1.set(Test2.getDeviceID());
		Test2.configNominalOutputVoltage(+0.0f, -0.0f);
		Test2.configPeakOutputVoltage(+12.0f, 0.0f);
		Test2.setProfile(0);
		Test2.setF((1 * 1023) / (-13000 * 4096.0 / 600.0));
		Test2.setP((0.6 * 1023) / -500);
		Test2.setI(0);
		Test2.setD(0);
	}

	// Sets the default command to run during teleop to joystick driving.
	public void initDefaultCommand() {
		setDefaultCommand(new Shoot());
	}

	// Gets joysticks input and calls the drive function with arguments.
	public void shoot() {
		Test3.set(-0.75);
		Test2.changeControlMode(TalonControlMode.Speed);
		Test2.set(-5000);
		SmartDashboard.putNumber("Speed", Test2.getSpeed());
	}
}