package org.usfirst.frc.team6217.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.usfirst.frc.team6217.robot.Robot;
import org.usfirst.frc.team6217.robot.RobotMap;

/*
 */
public class DriveTrain extends Subsystem {

	private double speedOfY =0;
	private double speedOfZ =0;
	private double speedOfX =0;
	private double lastSpeedOfY = 0;
	private double lastSpeedOfX = 0;
	private double lastSpeedOfZ = 0;
	
//	private WPI_VictorSPX _SPX_Left1 = new WPI_VictorSPX(RobotMap.VICTOR_SPX_DRIVE_LEFT1);
//	private WPI_VictorSPX _SPX_Left2 = new WPI_VictorSPX(RobotMap.VICTOR_SPX_DRIVE_LEFT2);
//	private WPI_VictorSPX _SPX_Right1 = new WPI_VictorSPX(RobotMap.VICTOR_SPX_DRIVE_RIGHT1);
//	private WPI_VictorSPX _SPX_Right2 = new WPI_VictorSPX(RobotMap.VICTOR_SPX_DRIVE_RIGHT2);

	private VictorSP _SPX_Left1 = new VictorSP(0);
	private VictorSP _SPX_Left2 = new VictorSP(1);
	private VictorSP _SPX_Right1 = new VictorSP(2);
	private VictorSP _SPX_Right2 = new VictorSP(3);

	private SpeedControllerGroup _leftSide = new SpeedControllerGroup(_SPX_Left1,_SPX_Left2);
	private SpeedControllerGroup _rightSide = new SpeedControllerGroup(_SPX_Right1, _SPX_Right2);
	private DifferentialDrive _driveTrain = new DifferentialDrive(_leftSide, _rightSide);
	
	private ADXRS450_Gyro _gyro = new ADXRS450_Gyro();
	private Accelerometer _accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
	private Encoder enc = new Encoder(0, 1, false, Encoder.EncodingType.k4X);

	private double _accer = 0;
	private double _lastAccer = 0;

	public void initDefaultCommand() {
        // Set the default command for a subsystem here. 
	}
    public DriveTrain() {
    	enc.setDistancePerPulse((Math.PI*6)/(20*10.71));
    }
	public boolean signbit(double xSignbit) {
		if (xSignbit >= 0) {
			return true;
		}
		else {
			return false;
		}
	}
    
	
    public void resetGyro() {
    	_gyro.reset();
    }
    
    public void calibrateGyro() {
    	_gyro.calibrate();
    }
    
	public double getGyroValue() {
		
		return _gyro.getAngle();
	}
	
	public double getEncoderValue() {
		System.out.println(enc.getRaw());
		return enc.getDistance();
	}
	
	public void resetEncoder() {
		enc.reset();
	}
	
	public double getAccerationOfX() {
    	SmartDashboard.putNumber("Acceration Value of X axle", _accel.getX());
		return _accel.getX();
	}
	
	public double getAccerationOfY() {
    	SmartDashboard.putNumber("Acceration Value Of Y axle", _accel.getY());
		return _accel.getY();
	}
	
	public double getAccerationOfZ() {
    	SmartDashboard.putNumber("Acceration Value Of Z axle", _accel.getZ());
		return _accel.getZ();
	}
	
	public void tankDrive(double rPower, double lPower) {
		_rightSide.set(rPower);
		_leftSide.set(lPower);
	}
	
	public void arcadeDrive(double xDir, double yDir ,double zRotation ,double governer , boolean squaredInputs) {
    	
    	
    	//Dead Zone
    	xDir = (Math.abs(xDir) > 0.10 ? xDir : 0.0);
    	yDir = (Math.abs(yDir) > 0.10 ? yDir : 0.0);
    	zRotation = (Math.abs(zRotation) > 0.15 ? zRotation :0.0);
    	
    	//Changing the speed according to the dial on the Joystick
    	xDir *= (1-governer);
    	//This is axis needs to be inverted
    	yDir *= -(1-governer);
    	zRotation *= (1-governer);
    	
	
    	//Limiting the turning speed to 50%
    	if (zRotation > RobotMap.MAXTURNINGSPEED){
    		zRotation = RobotMap.MAXTURNINGSPEED;
    	}
    	else if (zRotation < -(RobotMap.MAXTURNINGSPEED)){
    		zRotation = -(RobotMap.MAXTURNINGSPEED);
    	}

    	if (xDir > RobotMap.MAXTURNINGSPEED){
    		xDir = RobotMap.MAXTURNINGSPEED;
    	}
    	else if (xDir < -RobotMap.MAXTURNINGSPEED){
    		xDir = -(RobotMap.MAXTURNINGSPEED);
    	}
    	
    	//Acceleration of z
    	//Checking if the Sign of last Speed of z is a different sign of request speed or zRotation
    	if (signbit(zRotation) != signbit(lastSpeedOfZ)){
    		lastSpeedOfZ = 0;
    	}
    	//Do the Acceleration of turning
    	if (zRotation == 0){
    		speedOfZ = 0;
    	}
    	else if (Math.abs(zRotation) <= Math.abs(lastSpeedOfZ)){
    		speedOfZ = zRotation;
    	}
    	else if (Math.abs(zRotation) > Math.abs(lastSpeedOfZ)){
    		speedOfZ = lastSpeedOfZ + (zRotation * RobotMap.PERCENT_ACCEL);
    	}
    	
    	//Acceleration of Y
    	//Checking if the Sign of last Speed of Y is a different sign of request speed or ydir
    	if (signbit(yDir) != signbit(lastSpeedOfY)){
    		lastSpeedOfY = 0;
    	}
    	//Do the Acceleration of turning
    	if (yDir == 0){
    		speedOfY = 0;
    	}
    	else if (Math.abs(yDir) <= Math.abs(lastSpeedOfY)){
    		speedOfY = yDir;
    	}
    	else if (Math.abs(yDir) > Math.abs(lastSpeedOfY)){
    		speedOfY =  lastSpeedOfY + (yDir * RobotMap.PERCENT_ACCEL);
    	}

    	//Acceleration of x
    	//Checking if the Sign of last Speed of z is a different sign of request speed or zRotation
    	if (signbit(xDir) != signbit(lastSpeedOfX)){
    		lastSpeedOfX = 0;
    	}
    	//Do the Acceleration of turning
    	if (xDir == 0){
    	}
    	else if (Math.abs(xDir) <= Math.abs(lastSpeedOfX)){
    		speedOfX = xDir;
    	}
    	else if (Math.abs(xDir) > Math.abs(lastSpeedOfX)){
    		speedOfX = lastSpeedOfX + (xDir * RobotMap.PERCENT_ACCEL);
    	}
    	
    	//Setting the Speed
    	_driveTrain.arcadeDrive(speedOfY, speedOfZ, squaredInputs);
    	
    	//Setting the lastSpeed
    	lastSpeedOfZ = speedOfZ;
    	lastSpeedOfY = speedOfY;
    	lastSpeedOfX = speedOfX;
    	
    	//Display acceration on smartdashboard
    	_accer = ((Robot.driveTrain.getAccerationOfY()*32*.1)+(_lastAccer*.9));
    	SmartDashboard.putNumber("Acceration Value Of Z axle Average", Math.abs(_accer));
    	_accer = _lastAccer;
	}
}

