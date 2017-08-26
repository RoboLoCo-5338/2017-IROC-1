package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.SwerveDriveWithJoysticks;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {
	private final CANTalon DRIVESTEERING1 = new CANTalon(11);
	private final CANTalon DRIVEMOTOR1 = new CANTalon(12);
	private final CANTalon DRIVESTEERING2 = new CANTalon(21);
	private final CANTalon DRIVEMOTOR2 = new CANTalon(22);
	private final CANTalon DRIVESTEERING3 = new CANTalon(31);
	private final CANTalon DRIVEMOTOR3 = new CANTalon(32);
	private final CANTalon DRIVESTEERING4 = new CANTalon(41);
	private final CANTalon DRIVEMOTOR4 = new CANTalon(42);

	private final AnalogInput ENCODER1 = new AnalogInput(1);
	private final AnalogInput ENCODER2 = new AnalogInput(2);
	private final AnalogInput ENCODER3 = new AnalogInput(3);
	private final AnalogInput ENCODER4 = new AnalogInput(4);

	public DriveTrain() {
		super();
		AnalogInput.setGlobalSampleRate(62500);
		ENCODER1.setOversampleBits(2);
		ENCODER1.setAverageBits(2);
		ENCODER2.setOversampleBits(2);
		ENCODER2.setAverageBits(2);
		ENCODER3.setOversampleBits(2);
		ENCODER3.setAverageBits(2);
		ENCODER4.setOversampleBits(2);
		ENCODER4.setAverageBits(2);
	}

	public static void initSensors() {
		
	}
	
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new SwerveDriveWithJoysticks());
	}
	
	
	
	public void drive(OI oi) {
		if (oi.get(OI.Button.TEST1)) {
			double amt = oi.getLeft("Y");
			drive(amt, 0, amt, 0, amt, 0, amt, 0);
		} else if (oi.get(OI.Button.TEST2)) {
			drive(0, 0.25, 0, 0.25, 0, 0.25, 0, 0.25);
		} else {
			drive(0, 0, 0, 0, 0, 0, 0, 0);
		}
		
		double input = oi.getDirectionDegrees(0);
		double output = (4096.0/360.0) * input;
		
		
		SmartDashboard.putNumber("ENCODER1", ENCODER1.getAverageValue());
		SmartDashboard.putNumber("ENCODER2", ENCODER2.getAverageValue());
		SmartDashboard.putNumber("ENCODER3", ENCODER3.getAverageValue());
		SmartDashboard.putNumber("ENCODER4", ENCODER4.getAverageValue());
	}

	public void drive(double motor1, double steering1, double motor2, double steering2, double motor3, double steering3,
			double motor4, double steering4) {
		DRIVESTEERING1.set(steering1);
		DRIVESTEERING2.set(steering2);
		DRIVESTEERING3.set(steering3);
		DRIVESTEERING4.set(steering4);
		DRIVEMOTOR1.set(motor1);
		DRIVEMOTOR2.set(motor2);
		DRIVEMOTOR3.set(motor3);
		DRIVEMOTOR4.set(motor4);
	}
}