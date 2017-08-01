package Loops;

import Subsystems.Pidgeon;
import Subsystems.Swerve;
import Subsystems.Turret;
import Utilities.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class RobotStateEstimator implements Loop{
	static RobotStateEstimator instance = new RobotStateEstimator();
	
	public static RobotStateEstimator getInstance(){
		return instance;
	}
	
	RobotStateEstimator() {
    }
	
	Swerve swerve = Swerve.getInstance();
	Turret turret = Turret.getInstance();
	Pidgeon pidgey = Pidgeon.getInstance();
	double encoderPrevDistance = 0;
	
	@Override
	public void onStart(){
		encoderPrevDistance = swerve.rearRight.getEncoderDistanceFeet();
	}
	
	@Override
	public void onLoop(){
		double time = Timer.getFPGATimestamp();
		double encoderDistance = swerve.rearRight.getEncoderDistanceFeet();
		Rotation2d pidgeonAngle = Rotation2d.fromDegrees(pidgey.getAngle());
		Rotation2d turretAngle = Rotation2d.fromDegrees(turret.getFieldRelativeAngle());
		
	}
	
	@Override
	public void onStop(){
		
	}
}
