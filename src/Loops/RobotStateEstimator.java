package Loops;

import Subsystems.Pidgeon;
import Subsystems.RobotState;
import Subsystems.Swerve;
import Subsystems.Turret;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class RobotStateEstimator implements Loop{
	static RobotStateEstimator instance = new RobotStateEstimator();
	
	public static RobotStateEstimator getInstance(){
		return instance;
	}
	
	RobotStateEstimator() {
    }
	
	RobotState robotState = RobotState.getInstance();
	Swerve swerve = Swerve.getInstance();
	Turret turret = Turret.getInstance();
	Pidgeon pidgey = Pidgeon.getInstance();
	double encoderPrevDistance = 0;
	
	@Override
	public void onStart(){
		
	}
	
	@Override
	public void onLoop(){
		double time = Timer.getFPGATimestamp();
		double robotX = swerve.getX();
		double robotY = swerve.getY();
		Rotation2d pidgeonAngle = Rotation2d.fromDegrees(-pidgey.getAngle() + 90);
		Rotation2d turretAngle = Rotation2d.fromDegrees(-turret.getAngle());
		RigidTransform2d odometry = new RigidTransform2d(new Translation2d(robotX, robotY), pidgeonAngle);
		robotState.addObservations(time, odometry, turretAngle);
	}
	
	@Override
	public void onStop(){
		
	}
}
