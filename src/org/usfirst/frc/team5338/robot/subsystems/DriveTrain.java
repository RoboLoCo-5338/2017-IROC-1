package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.SwerveDriveWithJoysticks;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem {
	// Creates the eight CANTalon motor controller objects.
	private final CANTalon DRIVESTEERING1 = new CANTalon(11);
	private final CANTalon DRIVEMOTOR1 = new CANTalon(12);
	private final CANTalon DRIVESTEERING2 = new CANTalon(21);
	private final CANTalon DRIVEMOTOR2 = new CANTalon(22);
	private final CANTalon DRIVESTEERING3 = new CANTalon(31);
	private final CANTalon DRIVEMOTOR3 = new CANTalon(32);
	private final CANTalon DRIVESTEERING4 = new CANTalon(41);
	private final CANTalon DRIVEMOTOR4 = new CANTalon(42);

	// Creates the four AnalogInput objects for the encoders.
	private final AnalogInput ENCODER1 = new AnalogInput(0);
	private final AnalogInput ENCODER2 = new AnalogInput(1);
	private final AnalogInput ENCODER3 = new AnalogInput(2);
	private final AnalogInput ENCODER4 = new AnalogInput(3);

	double[] centers = { 1.539, 4.312, 0.529, 0.644 }; // don't use.

	// DriveTrain object constructor which configures encoders and reverses output
	// of backwards motors.
	public DriveTrain() {
		super();
		ENCODER1.setOversampleBits(4);
		ENCODER1.setAverageBits(4);
		ENCODER2.setOversampleBits(4);
		ENCODER2.setAverageBits(4);
		ENCODER3.setOversampleBits(4);
		ENCODER3.setAverageBits(4);
		ENCODER4.setOversampleBits(4);
		ENCODER4.setAverageBits(4);
		// Needs to reverse backwards motors.
	}

	// Sets the default command to run during teleop to joystick driving.
	public void initDefaultCommand() {
		setDefaultCommand(new SwerveDriveWithJoysticks());
	}

	public void drive(OI oi) {

		moveto(70.0);

		SmartDashboard.putNumber("ENCODER1", ENCODER1.getVoltage());
		SmartDashboard.putNumber("ENCODER2", ENCODER2.getVoltage());
		SmartDashboard.putNumber("ENCODER3", ENCODER3.getVoltage());
		SmartDashboard.putNumber("ENCODER4", ENCODER4.getVoltage());

	}

	public void moveto(double dir) {

		if (ENCODER1.getAverageVoltage() < centers[0] - 0.1)
			DRIVESTEERING1.set(-.25);
		else if (ENCODER1.getAverageVoltage() > centers[0] + 0.1)
			DRIVESTEERING1.set(.25);
		else
			DRIVESTEERING1.set(0);
		if (ENCODER2.getAverageVoltage() < centers[1] - 0.1)
			DRIVESTEERING2.set(-.25);
		else if (ENCODER2.getAverageVoltage() > centers[1] + 0.1)
			DRIVESTEERING2.set(.25);
		else
			DRIVESTEERING2.set(0);
		if (ENCODER3.getAverageVoltage() < centers[2] - 0.1)
			DRIVESTEERING3.set(-.25);
		else if (ENCODER3.getAverageVoltage() > centers[2] + 0.1)
			DRIVESTEERING3.set(.25);
		else
			DRIVESTEERING3.set(0);
		if (ENCODER4.getAverageVoltage() < centers[3] - 0.1)
			DRIVESTEERING4.set(-.25);
		else if (ENCODER4.getAverageVoltage() > centers[3] + 0.1)
			DRIVESTEERING4.set(.25);
		else
			DRIVESTEERING4.set(0);

	}

	// Sets output of CANTalons based on the double arguments.
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
	public double getEncoderVal(AnalogInput encoder) {
		int[] defaults = {52014,55990,47285,8454}; 
		if (encoder.equals(ENCODER1)) { 
			return (((encoder.getAverageValue()+defaults[0])/65535.0)*360.0)%360;	
		}
		else if (encoder.equals(ENCODER2)) { 
			return (((encoder.getAverageValue()+defaults[1])/65535.0)*360.0)%360;	
		}
		else if (encoder.equals(ENCODER3)) { 
			return (((encoder.getAverageValue()+defaults[2])/65535.0)*360.0)%360;	
		}
		else if (encoder.equals(ENCODER4)) { 
			return (((encoder.getAverageValue()+defaults[3])/65535.0)*360.0)%360;	
		}
		else {
			return -1.0;
		}
	}
	
}