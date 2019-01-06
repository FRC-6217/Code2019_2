package org.usfirst.frc.team6217.robot.subsystems;

import org.usfirst.frc.team6217.robot.commands.JoystickDrive;

import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
	private WheelDrive backRight = new WheelDrive (47, 44, 0);
	private WheelDrive backLeft = new WheelDrive (42, 41, 2);
	private WheelDrive frontRight = new WheelDrive (45, 46, 4);
	private WheelDrive frontLeft = new WheelDrive (40, 43, 6);

	private SwerveDriveClass swerveDrive = new SwerveDriveClass (backRight, backLeft, frontRight, frontLeft);

    public void initDefaultCommand() {
    	setDefaultCommand(new JoystickDrive());

    }
    public void Drive(double x, double y, double z) {
    	swerveDrive.drive (x, y, z);
    }
}

