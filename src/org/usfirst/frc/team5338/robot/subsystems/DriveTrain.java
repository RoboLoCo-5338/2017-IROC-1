package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.commands.SwerveDriveWithJoysticks;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
    public final CANTalon DRIVEL1 = new CANTalon(4);
    public final CANTalon DRIVEL2 = new CANTalon(3);
    public final CANTalon DRIVER1 = new CANTalon(2);
    public final CANTalon DRIVER2 = new CANTalon(1);

    public DriveTrain() {
	super();
    }

    @Override
    public void initDefaultCommand() {
    setDefaultCommand(new SwerveDriveWithJoysticks());
    }

    public void drive(OI oi) {
    }

    public void drive(double left, double right) {
    }
}
