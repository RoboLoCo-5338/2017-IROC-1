package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.SwerveDriveWithJoysticks;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;
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

	private final PIDController pid1 = new PIDController(0.1025*0.8, 0, 0.11666666667/8, new EncoderPID(1), DRIVESTEERING1, .001);
	private final PIDController pid2 = new PIDController(0.1025*0.8, 0, 0.11666666667/8, new EncoderPID(2), DRIVESTEERING2, .001);
	private final PIDController pid3 = new PIDController(0.1025*0.8, 0, 0.11666666667/8, new EncoderPID(3), DRIVESTEERING3, .001);
	private final PIDController pid4 = new PIDController(0.1025*0.8, 0, 0.11666666667/8, new EncoderPID(4), DRIVESTEERING4, .001);

	double[] centers = {52456.0, 56494.0, 47344.0, 9080.0};

	// DriveTrain object constructor which configures encoders and reverses
	// output
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
		DRIVESTEERING1.setInverted(true);
		DRIVESTEERING2.setInverted(true);
		DRIVESTEERING3.setInverted(true);
		DRIVESTEERING4.setInverted(true);

		pid1.setInputRange(0, 360);
		pid1.setContinuous(true);
		pid1.setOutputRange(-1, 1);
		pid1.setSetpoint(getEncoderVal(1));
		pid1.setAbsoluteTolerance(0.25);
		pid1.enable();

		pid2.setInputRange(0, 360);
		pid2.setContinuous(true);
		pid2.setOutputRange(-1, 1);
		pid2.setSetpoint(getEncoderVal(2));
		pid2.setAbsoluteTolerance(0.25);
		pid2.enable();

		pid3.setInputRange(0, 360);
		pid3.setContinuous(true);
		pid3.setOutputRange(-1, 1);
		pid3.setSetpoint(getEncoderVal(3));
		pid3.setAbsoluteTolerance(0.25);
		pid3.enable();

		pid4.setInputRange(0, 360);
		pid4.setContinuous(true);
		pid4.setOutputRange(-1, 1);
		pid4.setSetpoint(getEncoderVal(4));
		pid4.setAbsoluteTolerance(0.25);
		pid4.enable();

	}

	// Sets the default command to run during teleop to joystick driving.
	public void initDefaultCommand() {
		setDefaultCommand(new SwerveDriveWithJoysticks());
	}

	public void drive(OI oi) {
		double angle = oi.getLeft('A') + 180;
		if(angle > 0)
		{
			double magnitude = oi.getLeft('M') * oi.getRight('T');
			drive(magnitude, angle, magnitude, angle, magnitude, angle, magnitude, angle);
		}
		else
		{
			double magnitude = 0;
			angle = pid1.getSetpoint();
			drive(magnitude, angle, magnitude, angle, magnitude, angle, magnitude, angle);
		}
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
		int[] defaults = { 52021, 47665, 56888, 8903 };
		if (encoder == 1) {
			return (((ENCODER1.getAverageValue() - defaults[0] + 65535) % 65536) / 65535.0) * 360.0;
		} else if (encoder == 2) {
			return (((ENCODER2.getAverageValue() - defaults[1] + 65535) % 65536) / 65535.0) * 360.0;
		} else if (encoder == 3) {
			return (((ENCODER3.getAverageValue() - defaults[2] + 65535) % 65536) / 65535.0) * 360.0;
		} else if (encoder == 4) {
			return (((ENCODER4.getAverageValue() - defaults[3] + 65535) % 65536) / 65535.0) * 360.0;
		} else {
			return -1.0;
		}
	}

	double scaleInput(double dVal) {
		double[] scaleArray = new double[360];
		for (int i = 0; i < scaleArray.length; i += 2) {
			scaleArray[i] = (double) i;
		}

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * scaleArray.length);
		if (index < 0) {
			index = -index;
		} else if (index > 360) {
			index = 360;
		}

		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
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

}