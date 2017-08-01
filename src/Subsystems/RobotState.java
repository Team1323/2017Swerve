package Subsystems;

import Utilities.Constants;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.Translation2d;

public class RobotState {
	private static RobotState instance = new RobotState();
	
	public static RobotState getInstance(){
		return instance;
	}
	
	public static final int kObservationBufferSize = 100;
    public static final double kMaxTargetAge = 0.4;
    
    public static final RigidTransform2d kVehicleToTurretFixed = new RigidTransform2d(
            new Translation2d(0, 0),
            Rotation2d.fromDegrees(0));

    public static final RigidTransform2d kTurretRotatingToCamera = new RigidTransform2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), new Rotation2d());
    
    
}
