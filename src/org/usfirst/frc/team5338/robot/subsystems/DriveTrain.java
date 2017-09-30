package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.SwerveDriveWithJoysticks;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
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

	private final PIDController pid1 = new PIDController(0.055, 0, 0, new EncoderPID(0),
			DRIVESTEERING1, 0.001);
	private final PIDController pid2 = new PIDController(0.055, 0, 0, new EncoderPID(1),
			DRIVESTEERING2, 0.001);
	private final PIDController pid3 = new PIDController(0.055, 0, 0, new EncoderPID(2),
			DRIVESTEERING3, 0.001);
	private final PIDController pid4 = new PIDController(0.055, 0, 0, new EncoderPID(3),
			DRIVESTEERING4, 0.001);
	
	private Vector wheel1 = new Vector(0, 0);
	private Vector wheel2 = new Vector(0, 0);
	private Vector wheel3 = new Vector(0, 0);
	private Vector wheel4 = new Vector(0, 0);
	
	private static final AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) (200));

	// DriveTrain object constructor which configures encoders and reverses
	// output
	// of backwards motors.
	public DriveTrain() {
		super();
		while (ahrs.isCalibrating() || ahrs.isMagnetometerCalibrated()) {
		}
		ahrs.reset();
		ahrs.zeroYaw();
		
		ENCODER1.setOversampleBits(4);
		ENCODER1.setAverageBits(4);
		ENCODER2.setOversampleBits(4);
		ENCODER2.setAverageBits(4);
		ENCODER3.setOversampleBits(4);
		ENCODER3.setAverageBits(4);
		ENCODER4.setOversampleBits(4);
		ENCODER4.setAverageBits(4);

		DRIVEMOTOR1.setInverted(false);
		DRIVEMOTOR2.setInverted(false);
		DRIVEMOTOR3.setInverted(false);
		DRIVEMOTOR4.setInverted(false);
		DRIVESTEERING1.setInverted(true);
		DRIVESTEERING2.setInverted(true);
		DRIVESTEERING3.setInverted(true);
		DRIVESTEERING4.setInverted(true);

		pid1.setInputRange(-180, 180);
		pid1.setContinuous(true);
		pid1.setOutputRange(-1, 1);
		pid1.setSetpoint(getEncoderVal(0));
		pid1.setAbsoluteTolerance(0.25);
		pid1.enable();

		pid2.setInputRange(-180, 180);
		pid2.setContinuous(true);
		pid2.setOutputRange(-1, 1);
		pid2.setSetpoint(getEncoderVal(1));
		pid2.setAbsoluteTolerance(0.25);
		pid2.enable();

		pid3.setInputRange(-180, 180);
		pid3.setContinuous(true);
		pid3.setOutputRange(-1, 1);
		pid3.setSetpoint(getEncoderVal(2));
		pid3.setAbsoluteTolerance(0.25);
		pid3.enable();

		pid4.setInputRange(-180, 180);
		pid4.setContinuous(true);
		pid4.setOutputRange(-1, 1);
		pid4.setSetpoint(getEncoderVal(3));
		pid4.setAbsoluteTolerance(0.25);
		pid4.enable();

	}

	// Sets the default command to run during teleop to joystick driving.
	public void initDefaultCommand() {
		setDefaultCommand(new SwerveDriveWithJoysticks());
	}

	public void drive(OI oi) {
		double angle = oi.getLeft('A');
		if (angle == 999)
		{
			angle = pid1.getSetpoint();
			drive(0, angle, 0, angle, 0, angle, 0, angle);
			return;
		}
		double magnitude = oi.getLeft('M') * oi.getRight('T');
		wheel1.setAngle(angle);
		wheel2.setAngle(angle);
		wheel3.setAngle(angle);
		wheel4.setAngle(angle);
		wheel1.setMagnitude(magnitude);
		wheel2.setMagnitude(magnitude);
		wheel3.setMagnitude(magnitude);
		wheel4.setMagnitude(magnitude);
		
		//rotation stuffs here
		
		//		
//		double R = oi.getLeft('Z')  * oi.getRight('T');
//		double X = oi.getLeft('X') * oi.getRight('T');
//		double Y = oi.getLeft('Y') * oi.getRight('T');
//		double T = ahrs.getAngle();
//		
//		double temp = Y * Math.cos(T) + X * Math.sin(T);
//		X = -Y * Math.sin(T) + X * Math.cos(T);
//		Y = temp;
//		
//		double A = X - R * Math.sqrt(2) / 2;
//		double B = X + R * Math.sqrt(2) / 2;
//		double C = Y - R * Math.sqrt(2) / 2;
//		double D = Y + R * Math.sqrt(2) / 2;
//		
//		double ws1 = Math.sqrt(B * B + C * C);
//		double ws2 = Math.sqrt(B * B + D * D);
//		double ws3 = Math.sqrt(A * A + D * D);
//		double ws4 = Math.sqrt(A * A + C * C);
//		
//		double max = Math.max(Math.max(ws1, ws2), Math.max(ws3, ws4));
//		ws1 /= max;
//		ws2 /= max;
//		ws3 /= max;
//		ws4 /= max;
		
		//drive(ws1, -(-180 - Math.atan2(B, C) * 180 / Math.PI), ws2, 180 - Math.atan2(B, D) * 180 / Math.PI, ws3, -(180 - Math.atan2(A, D) * 180 / Math.PI), ws4, 180 + Math.atan2(A, C) * 180 / Math.PI);
		drive(wheel1.getMagnitude(), wheel1.getAngle(), wheel2.getMagnitude(), wheel2.getAngle(), wheel3.getMagnitude(), wheel3.getAngle(), wheel4.getMagnitude(), wheel4.getAngle());
		SmartDashboard.putNumber("ENCODER1", getEncoderVal(0));
		SmartDashboard.putNumber("ENCODER2", getEncoderVal(1));
		SmartDashboard.putNumber("ENCODER3", getEncoderVal(2));
		SmartDashboard.putNumber("ENCODER4", getEncoderVal(3));
		SmartDashboard.putNumber("ANGLE", angle);
	}

	// Sets output of CANTalons and PID based on the double arguments.
	public void drive(double motor1, double angle1, double motor2, double angle2, double motor3, double angle3,
			double motor4, double angle4) {
		pid1.setSetpoint(angle1);
		pid2.setSetpoint(angle2);
		pid3.setSetpoint(angle3);
		pid4.setSetpoint(angle4);
		DRIVEMOTOR1.set(motor1);
		DRIVEMOTOR2.set(motor2);
		DRIVEMOTOR3.set(motor3);
		DRIVEMOTOR4.set(motor4);
	}

	public double getEncoderVal(int encoder) {
		int[] defaults = {13125, 15481, 15812, 60876};
		if (encoder == 0) {
			return (((ENCODER1.getAverageValue() - defaults[0] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else if (encoder == 1) {
			return (((ENCODER2.getAverageValue() - defaults[1] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else if (encoder == 2) {
			return (((ENCODER3.getAverageValue() - defaults[2] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else if (encoder == 3) {
			return (((ENCODER4.getAverageValue() - defaults[3] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else {
			return -1.0;
		}
	}

	class EncoderPID implements PIDSource {
		public int encoder;
		public EncoderPID(int encoder) {
			this.encoder = encoder;
		}
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			ENCODER1.setPIDSourceType(pidSource);
		}
		@Override
		public PIDSourceType getPIDSourceType() {
			return ENCODER1.getPIDSourceType();
		}
		@Override
		public double pidGet() {
			return getEncoderVal(encoder);
		}
	}
	
	class Vector
	{
		private double magnitude;
		private double angle;
		public Vector(double m, double a)
		{
			magnitude = m;
			angle = a;
	}
		double getAngle()
		{
			return angle;
		}
		void setAngle(double a)
		{
			angle = a;
		}
		double getMagnitude()
		{
			return magnitude;
		}
		void setMagnitude(double m)
		{
			magnitude = m;
		}
	}
}