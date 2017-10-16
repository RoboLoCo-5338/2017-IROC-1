package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.Robot;
import org.usfirst.frc.team5338.robot.commands.SwerveDrive;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drivetrain extends Subsystem {
	// Creates the eight CANTalon motor controller objects.
	private final CANTalon DRIVE_STEERING_1 = new CANTalon(11);
	private final CANTalon DRIVE_MOTOR_1 = new CANTalon(12);
	private final CANTalon DRIVE_STEERING_2 = new CANTalon(21);
	private final CANTalon DRIVE_MOTOR_2 = new CANTalon(22);
	private final CANTalon DRIVE_STEERING_3 = new CANTalon(31);
	private final CANTalon DRIVE_MOTOR_3 = new CANTalon(32);
	private final CANTalon DRIVE_STEERING_4 = new CANTalon(41);
	private final CANTalon DRIVE_MOTOR_4 = new CANTalon(42);

	// Creates the four AnalogInput objects for the encoders.
	private final AnalogInput ENCODER_1 = new AnalogInput(0);
	private final AnalogInput ENCODER_2 = new AnalogInput(1);
	private final AnalogInput ENCODER_3 = new AnalogInput(2);
	private final AnalogInput ENCODER_4 = new AnalogInput(3);

	// Creates the four PIDController objects for the wheel modules.
	private final PIDController PID_1 = new PIDController(0.055, 0, 0, new EncoderPID(0), DRIVE_STEERING_1, 0.001);
	private final PIDController PID_2 = new PIDController(0.055, 0, 0, new EncoderPID(1), DRIVE_STEERING_2, 0.001);
	private final PIDController PID_3 = new PIDController(0.055, 0, 0, new EncoderPID(2), DRIVE_STEERING_3, 0.001);
	private final PIDController PID_4 = new PIDController(0.055, 0, 0, new EncoderPID(3), DRIVE_STEERING_4, 0.001);

	// Creates the four Vector objects for the wheel modules.
	private Vector wheel1 = new Vector(0, 0);
	private Vector wheel2 = new Vector(0, 0);
	private Vector wheel3 = new Vector(0, 0);
	private Vector wheel4 = new Vector(0, 0);

	// Creates the angle object for the direction.
	private double angle;

	// Creates the AHRS object for the NavX.
	private static final AHRS AHRS = new AHRS(SPI.Port.kMXP, (byte) (200));

	// DriveTrain object constructor which configures input for encoders, configures
	// NavX, configures output of motors, and configures the PIDs
	public Drivetrain() {
		super();
		while (AHRS.isCalibrating() || AHRS.isMagnetometerCalibrated()) {
		}
		AHRS.reset();
		AHRS.zeroYaw();
		for (AnalogInput encoder : new AnalogInput[] { ENCODER_1, ENCODER_2, ENCODER_3, ENCODER_4 }) {
			encoder.setOversampleBits(4);
			encoder.setAverageBits(4);
		}
		for (CANTalon controller : new CANTalon[] { DRIVE_MOTOR_1, DRIVE_MOTOR_2, DRIVE_MOTOR_3, DRIVE_MOTOR_4 }) {
			controller.setInverted(false);
		}
		for (CANTalon controller : new CANTalon[] { DRIVE_STEERING_1, DRIVE_STEERING_2, DRIVE_STEERING_3,
				DRIVE_STEERING_4 }) {
			controller.setInverted(true);
		}
		for (PIDController controller : new PIDController[] { PID_1, PID_2, PID_3, PID_4 }) {
			controller.setInputRange(-180, 180);
			controller.setContinuous(true);
			controller.setOutputRange(-1, 1);
			controller.setSetpoint(getEncoderVal(0));
			controller.setAbsoluteTolerance(0.25);
			controller.enable();
		}
	}

	// Sets the default command to run during teleop to joystick driving.
	public void initDefaultCommand() {
		setDefaultCommand(new SwerveDrive());
	}

	public void drive(OI oi) {
		double throttle = oi.get('T');
		double magnitude = oi.get('M') * throttle;
		double rotation = oi.get('Z') * throttle;
		if (Robot.oi.get(OI.Button.RESET_YAW_1) && Robot.oi.get(OI.Button.RESET_YAW_2)) {
			AHRS.zeroYaw();
		}
		double heading = AHRS.getYaw();

		if (magnitude != 0) {
			angle = oi.get('A');
		}
		double wheelAngle = (angle - heading);
		if (wheelAngle > 180) {
			wheelAngle -= 360;
		} else if (wheelAngle < -180) {
			wheelAngle += 360;
		}
		for (Vector module : new Vector[] { wheel1, wheel2, wheel3, wheel4 }) {
			module.setMagnitude(0.0);
		}
		if (!Robot.oi.get(OI.Button.NO_TRANSLATION)) {
			for (Vector module : new Vector[] { wheel1, wheel2, wheel3, wheel4 }) {
				module.setAngleMagnitude(wheelAngle, magnitude);
			}
		} else {
			rotation = oi.get('Z') * throttle / 2;
		}
		if (!Robot.oi.get(OI.Button.NO_ROTATION)) {
			wheel1.add(new Vector(rotation, 45));
			wheel2.add(new Vector(rotation, 135));
			wheel3.add(new Vector(rotation, -45));
			wheel4.add(new Vector(rotation, -135));
		}
		if (magnitude == 0 && rotation == 0) {
			wheel1.setAngleMagnitude(PID_1.getSetpoint(), 0);
			wheel2.setAngleMagnitude(PID_2.getSetpoint(), 0);
			wheel3.setAngleMagnitude(PID_3.getSetpoint(), 0);
			wheel4.setAngleMagnitude(PID_4.getSetpoint(), 0);
		}
		normalize(wheel1, wheel2, wheel3, wheel4);
		drive(wheel1, wheel2, wheel3, wheel4);
	}

	public void normalize(Vector one, Vector two, Vector three, Vector four) {
		double max = Math.max(Math.max(one.getMagnitude(), two.getMagnitude()),
				Math.max(three.getMagnitude(), four.getMagnitude()));
		if (max > 1) {
			one.setMagnitude(one.getMagnitude() / max);
			two.setMagnitude(two.getMagnitude() / max);
			three.setMagnitude(three.getMagnitude() / max);
			four.setMagnitude(four.getMagnitude() / max);
		}
	}

	// Sets output of CANTalons and target of PIDControllers based on the double
	// arguments.
	public void drive(Vector one, Vector two, Vector three, Vector four) {
		PID_1.setSetpoint(one.getAngle());
		PID_2.setSetpoint(two.getAngle());
		PID_3.setSetpoint(three.getAngle());
		PID_4.setSetpoint(four.getAngle());
		DRIVE_MOTOR_1.set(one.getMagnitude());
		DRIVE_MOTOR_2.set(two.getMagnitude());
		DRIVE_MOTOR_3.set(three.getMagnitude());
		DRIVE_MOTOR_4.set(four.getMagnitude());
	}

	public double getEncoderVal(int encoder) {
		final int[] ZERO_CONSTANTS = { 13190, 16759, 16000, 51650 };
		if (encoder == 0) {
			return (((ENCODER_1.getAverageValue() - ZERO_CONSTANTS[0] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else if (encoder == 1) {
			return (((ENCODER_2.getAverageValue() - ZERO_CONSTANTS[1] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else if (encoder == 2) {
			return (((ENCODER_3.getAverageValue() - ZERO_CONSTANTS[2] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else if (encoder == 3) {
			return (((ENCODER_4.getAverageValue() - ZERO_CONSTANTS[3] + 65535) % 65536) / 65535.0) * -360.0 + 180;
		} else {
			return -1.0;
		}
	}

	class EncoderPID implements PIDSource {
		public int encoderIndex;

		public EncoderPID(int encoder) {
			encoderIndex = encoder;
		}

		public void setPIDSourceType(PIDSourceType pidSource) {
			ENCODER_1.setPIDSourceType(pidSource);
		}

		public PIDSourceType getPIDSourceType() {
			return ENCODER_1.getPIDSourceType();
		}

		public double pidGet() {
			return getEncoderVal(encoderIndex);
		}
	}

	public class Vector {
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
			double otherAngleRadians = Math.toRadians(other.angle);
			double thisAngleRadians = Math.toRadians(angle);
			double rX = magnitude * Math.cos(thisAngleRadians) + other.magnitude * Math.cos(otherAngleRadians);
			double rY = magnitude * Math.sin(thisAngleRadians) + other.magnitude * Math.sin(otherAngleRadians);
			magnitude = Math.sqrt((Math.pow(rX, 2) + Math.pow(rY, 2)));
			angle = Math.toDegrees(Math.atan2(rY, rX));
		}
	}
}