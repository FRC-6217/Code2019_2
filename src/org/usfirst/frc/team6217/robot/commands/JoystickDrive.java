package org.usfirst.frc.team6217.robot.commands;

import org.usfirst.frc.team6217.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickDrive extends Command {
	
    public JoystickDrive() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.m_driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.m_driveTrain.Drive (Robot.m_oi.joystick.getRawAxis (1), Robot.m_oi.joystick.getRawAxis (0), Robot.m_oi.joystick.getRawAxis (2));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	isFinished();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
