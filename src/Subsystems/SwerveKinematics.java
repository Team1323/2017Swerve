package Subsystems;

import Utilities.Constants;
import Utilities.Util;

public class SwerveKinematics {
	
	private double frontRightWheelSpeed = 0.0;
	private double frontLeftWheelSpeed = 0.0;
	private double rearLeftWheelSpeed = 0.0;
	private double rearRightWheelSpeed = 0.0;
	
	private double frontRightSteeringAngle = 0.0;
	private double frontLeftSteeringAngle = 0.0;
	private double rearLeftSteeringAngle = 0.0;
	private double rearRightSteeringAngle = 0.0;
	
	public void calculate(double x, double y, double rotate){
		double A = x - rotate * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
	    double B = x + rotate * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
	    double C = y - rotate * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
	    double D = y + rotate * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
	    
	    frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
	    frontLeftWheelSpeed  = Math.sqrt((B * B) + (D * D));
	    rearLeftWheelSpeed   = Math.sqrt((A * A) + (D * D));
	    rearRightWheelSpeed  = Math.sqrt((A * A) + (C * C));
	    double max = frontRightWheelSpeed;
	    max = Util.normalize(max, frontLeftWheelSpeed);
	    max = Util.normalize(max, rearLeftWheelSpeed);
	    max = Util.normalize(max, rearRightWheelSpeed);
	    if(max > 1.0){
	    	frontRightWheelSpeed /= max;
	        frontLeftWheelSpeed /= max;
	        rearLeftWheelSpeed /= max;
	        rearRightWheelSpeed /= max;
	    }
	    
	    frontRightSteeringAngle = Math.atan2(B, C)*180/Math.PI; 
	    frontLeftSteeringAngle = Math.atan2(B, D)*180/Math.PI;
	    rearLeftSteeringAngle = Math.atan2(A, D)*180/Math.PI;
	    rearRightSteeringAngle = Math.atan2(A, C)*180/Math.PI;
	}
	
	public double frWheelSpeed(){
		return frontRightWheelSpeed;
	}
	public double flWheelSpeed(){
		return frontLeftWheelSpeed;
	}
	public double rlWheelSpeed(){
		return rearLeftWheelSpeed;
	}
	public double rrWheelSpeed(){
		return rearRightWheelSpeed;
	}
	
	public double frSteeringAngle(){
		return frontRightSteeringAngle;
	}
	public double flSteeringAngle(){
		return frontLeftSteeringAngle;
	}
	public double rlSteeringAngle(){
		return rearLeftSteeringAngle;
	}
	public double rrSteeringAngle(){
		return rearRightSteeringAngle;
	}
}