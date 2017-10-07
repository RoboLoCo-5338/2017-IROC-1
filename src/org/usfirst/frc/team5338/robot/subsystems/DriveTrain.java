package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.Robot;
import org.usfirst.frc.team5338.robot.commands.SwerveDriveWithJoysticks;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
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

	private final PIDController pid1 = new PIDController(0.055, 0, 0, new EncoderPID(0), DRIVESTEERING1, 0.001);
	private final PIDController pid2 = new PIDController(0.055, 0, 0, new EncoderPID(1), DRIVESTEERING2, 0.001);
	private final PIDController pid3 = new PIDController(0.055, 0, 0, new EncoderPID(2), DRIVESTEERING3, 0.001);
	private final PIDController pid4 = new PIDController(0.055, 0, 0, new EncoderPID(3), DRIVESTEERING4, 0.001);

	private Vector wheel1 = new Vector(0, 0);
	private Vector wheel2 = new Vector(0, 0);
	private Vector wheel3 = new Vector(0, 0);
	private Vector wheel4 = new Vector(0, 0);

	private double angle;

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
		double throttle = oi.getLeft('T');
		double magnitude = oi.getLeft('M') * throttle;
		double rotation = oi.getLeft('Z') * throttle;
		double shooter = oi.getRight('T');
		double heading = ahrs.getYaw();

		if (magnitude != 0) {
			angle = oi.getLeft('A');
		}

		double wheelAngle = (angle - heading);
		if (wheelAngle > 180) {
			wheelAngle -= 360;
		} else if (wheelAngle < -180) {
			wheelAngle += 360;
		}
		
		wheel1.setMagnitude(0);
		wheel2.setMagnitude(0);
		wheel3.setMagnitude(0);
		wheel4.setMagnitude(0);
		
		if (!Robot.oi.get(OI.Button.NOTRANSLATION)) {
			wheel1.setAngleMagnitude(wheelAngle, magnitude);
			wheel2.setAngleMagnitude(wheelAngle, magnitude);
			wheel3.setAngleMagnitude(wheelAngle, magnitude);
			wheel4.setAngleMagnitude(wheelAngle, magnitude);
		}
		if (!Robot.oi.get(OI.Button.NOROTATION)) {
			wheel1.add(new Vector(rotation, 45));
			wheel2.add(new Vector(rotation, 135));
			wheel3.add(new Vector(rotation, -45));
			wheel4.add(new Vector(rotation, -135));
		}

		drive(wheel1, wheel2, wheel3, wheel4);
		
		SmartDashboard.putNumber("ENCODER 1", ENCODER1.getAverageValue());
		SmartDashboard.putNumber("ENCODER 2", ENCODER2.getAverageValue());
		SmartDashboard.putNumber("ENCODER 3", ENCODER3.getAverageValue());
		SmartDashboard.putNumber("ENCODER 4", ENCODER4.getAverageValue());
		SmartDashboard.putNumber("ANG:ENCODER 1", getEncoderVal(0));
		SmartDashboard.putNumber("ANG:ENCODER 2", getEncoderVal(1));
		SmartDashboard.putNumber("ANG:ENCODER 3", getEncoderVal(2));
		SmartDashboard.putNumber("ANG:ENCODER 4", getEncoderVal(3));
	}

	// Sets output of CANTalons and PID based on the double arguments.
	public void drive(Vector one, Vector two, Vector three, Vector four) {
		pid1.setSetpoint(one.getAngle());
		pid2.setSetpoint(two.getAngle());
		pid3.setSetpoint(three.getAngle());
		pid4.setSetpoint(four.getAngle());
		DRIVEMOTOR1.set(one.getMagnitude());
		DRIVEMOTOR2.set(two.getMagnitude());
		DRIVEMOTOR3.set(three.getMagnitude());
		DRIVEMOTOR4.set(four.getMagnitude());
	}

	public double getEncoderVal(int encoder) {
		int[] defaults = { 13190, 16759, 16000, 51650 };
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

	class Vector {
		private double magnitude;
		private double angle;

		public Vector(double m, double a) {
			magnitude = m;
			angle = a;

		}

		void setAngleMagnitude(double a, double m) {
			angle = a;
			magnitude = m;
		}

		double getAngle() {
			return angle;
		}

		void setAngle(double a) {
			angle = a;
		}

		double getMagnitude() {
			return magnitude;
		}

		void setMagnitude(double m) {
			magnitude = m;
		}

		void add(Vector other) {
			other = new Vector(other.getMagnitude(), Math.toRadians(other.getAngle()));
			this.setAngle(Math.toRadians(this.angle));
			double rX = (this.magnitude * Math.cos(this.angle)) + (other.getMagnitude() * Math.cos(other.getAngle()));
			double rY = (this.magnitude * Math.sin(this.angle)) + (other.getMagnitude() * Math.sin(other.getAngle()));
			this.magnitude = Math.sqrt((Math.pow(rX, 2) + Math.pow(rY, 2)));
			this.angle = Math.toDegrees(Math.atan2(rY, rX));
		}
	}
}