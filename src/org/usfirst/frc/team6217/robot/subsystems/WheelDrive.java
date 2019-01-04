package org.usfirst.frc.team6217.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class WheelDrive {
	private VictorSPX angleMotor;
	private VictorSPX speedMotor;
	private PIDController pidController;
	private final double MAX_VOLTS = 4.95;
	
	public WheelDrive (int angleMotor, int speedMotor, int encoder) {
	    this.angleMotor = new VictorSPX (angleMotor);
	    this.speedMotor = new VictorSPX (speedMotor);
	    //VictorSPX is not a subclass of PIDOutput;
	    pidController = new PIDController (1, 0, 0, new AnalogInput (encoder), (PIDOutput) this.angleMotor);

	    pidController.setOutputRange (-1, 1);
	    pidController.setContinuous ();
	    pidController.enable ();
	    
	}
	
	public void drive (double speed, double angle) {
	    speedMotor.set(ControlMode.PercentOutput, speed);

	    double setpoint = (angle * (MAX_VOLTS * 0.5)) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
	    if (setpoint < 0) {
	        setpoint = MAX_VOLTS + setpoint;
	    }
	    if (setpoint > MAX_VOLTS) {
	        setpoint = setpoint - MAX_VOLTS;
	    }

	    pidController.setSetpoint (setpoint);
	}
	

	
}
