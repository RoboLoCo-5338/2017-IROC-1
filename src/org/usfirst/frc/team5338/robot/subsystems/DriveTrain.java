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

	private final AnalogInput ENCODER1 = new AnalogInput(0);
	private final AnalogInput ENCODER2 = new AnalogInput(1);
	private final AnalogInput ENCODER3 = new AnalogInput(2);
	private final AnalogInput ENCODER4 = new AnalogInput(3);

	double[] centers = { 1.539, 4.312, 0.529, 0.644 };

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
		// if (oi.get(OI.Button.TEST1)) {
		// double amt = oi.getLeft("Y");
		// drive(amt, 0, amt, 0, amt, 0, amt, 0);
		// } else if (oi.get(OI.Button.TEST2)) {
		// drive(0, 0.25, 0, 0.25, 0, 0.25, 0, 0.25);
		// } else {
		// drive(0, 0, 0, 0, 0, 0, 0, 0);
		// }

		moveto(70.0);

		// double input = oi.getDirectionDegrees(1);
		// double output = (5.0 / 360.0) * input;
		// double amt = oi.getLeft("Y");
		// if (ENCODER1.getVoltage() < output)
		// drive(amt, 10, 0, 0, 0, 0, 0, 0);
		SmartDashboard.putNumber("ENCODER1", ENCODER1.getVoltage());
		SmartDashboard.putNumber("ENCODER2", ENCODER2.getVoltage());
		SmartDashboard.putNumber("ENCODER3", ENCODER3.getVoltage());
		SmartDashboard.putNumber("ENCODER4", ENCODER4.getVoltage());

		// double ec1 = ENCODER1.getAverageVoltage();
		// double ec1c = 2.5;
		// if (ec1 > ec1c + 0.01) {
		// DRIVESTEERING1.set(-0.1); //Left
		// }
		// else if (ec1 < ec1c - 0.01) {
		// DRIVESTEERING1.set(0.1); //Right
		// }
		// else {
		// DRIVESTEERING1.set(0);
		// }

	}

	public void moveto(double dir) {
		// SmartDashboard.putNumber("movedTo", 0);
		// double ec1 = ENCODER1.getVoltage() + 0.0;
		// ec1 = (ec1 * 72) + dir % 360.0;
		// // double ec2 = ENCODER2.getVoltage() + /*constant*/;
		// // ec2 = (ec2 * 72) + dir % 360.0;
		// // double ec3 = ENCODER3.getVoltage() + /*constant*/;
		// // ec3 = (ec3 * 72) + dir % 360.0;
		// // double ec4 = ENCODER4.getVoltage() + /*constant*/;
		// // ec4 = (ec4 * 72) + dir % 360.0;
		// if (ec1 > dir + 0.5) {
		// DRIVESTEERING1.set(-0.5); // Left
		// } else if (ec1 < dir - 0.5) {
		// DRIVESTEERING1.set(0.5); // Right
		// } else {
		// DRIVESTEERING1.set(0);
		// }
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