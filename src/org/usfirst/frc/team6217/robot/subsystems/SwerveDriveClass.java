package org.usfirst.frc.team6217.robot.subsystems;

//defining the class SwerveDrive;
public class SwerveDriveClass {
	
	//creating constants for the width and length;
	public final double L = LENGTH;
	public final double W = WIDTH;
	
	private WheelDrive backRight;
	private WheelDrive backLeft;
	private WheelDrive frontRight;
	private WheelDrive frontLeft;
	
	//creating a constructor called SwerveDriveClass defining its parameters;
	public SwerveDriveClass (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
	    this.backRight = backRight;
	    this.backLeft = backLeft;
	    this.frontRight = frontRight;
	    this.frontLeft = frontLeft;
	}
	
	public void drive (double x, double y, double z) {
		/*First we need to find the radius of the circle the robot is going to spin
		 * and for that we use the Pythagorean theorem and y *= -1 means y is assigned
		 * the value of y * -1 */
		double r = Math.sqrt ((L * L) + (W * W));
		y *= -1;
		
		//The next thing we do is assign the value a to the equation. a makes the robot go backwards
		//Note: The L/r part makes the code in radians not degrees
		double a = x - z * (L / r); //go backwards in the x direction;
		double b = x + z * (L / r);//go forwards in the y direction;
		double c = y - z * (W / r);//go left;
		double d = y + z * (W / r);//go right;
	    
		/*Now we do the Pythagorean theorem again because the robot will only go in a straight
		 * line so we find the hypotenuse using our above variables for all direction combos. */
		double backRightSpeed = Math.sqrt ((a * a) + (d * d));
	    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
	    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
	    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));
	    
	    /*In order to find the angles we need to turn based on the inputs of the controller
         * the code turns the tangent of the coordinates (x,y) into (radius,angle) to find the
         * angle that applies to our robot. Then it's divided by pi to turn from radians 
         * into degrees. */
	    double backRightAngle = Math.atan2 (a, d) / Math.PI;
	    double backLeftAngle = Math.atan2 (a, c) / Math.PI;
	    double frontRightAngle = Math.atan2 (b, d) / Math.PI;
	    double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
	    
	    /*Lastly the results of the above code are all plugged back in to be used later. */
	    backRight.drive (backRightSpeed, backRightAngle);
	    backLeft.drive (backLeftSpeed, backLeftAngle);
	    frontRight.drive (frontRightSpeed, frontRightAngle);
	    frontLeft.drive (frontLeftSpeed, frontLeftAngle);
	}
}
