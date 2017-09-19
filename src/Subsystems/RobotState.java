package Subsystems;


import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import Utilities.Constants;
import Utilities.InterpolatingDouble;
import Utilities.InterpolatingTreeMap;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.ShooterAimingParameters;
import Utilities.Translation2d;
import Vision.GoalTrack;
import Vision.GoalTrack.TrackReport;
import Vision.TargetInfo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
	private static RobotState instance = new RobotState();
	
	public static RobotState getInstance(){
		return instance;
	}
	
	public static final int kObservationBufferSize = 100;
    public static final double kMaxTargetAge = 0.4;
    
 // FPGATimestamp -> RigidTransform2d or Rotation2d
    protected InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> fieldToVehicle;
    protected RigidTransform2d.Delta vehicleVelocity;
    protected InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turretRotation;
    protected GoalTrack goalTrack;
    private int nextID;
    protected Rotation2d cameraPitchCorrection;
    protected Rotation2d cameraYawCorrection;
    protected double differentialHeight;
    double visionAngle = 0.0;
    double distanceToTarget = 0.0;
    public double getVisionAngle(){
    	return visionAngle;
    }
    boolean seesTarget = false;
    public boolean getTargetVisbility(){
    	return seesTarget;
    }
    public double getTargetDistance(){
    	return distanceToTarget;
    }
    
    protected RobotState() {
        reset(0, new RigidTransform2d(), new Rotation2d());
    }

    public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle,
            Rotation2d initial_turret_rotation) {
    	fieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
    	fieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
    	vehicleVelocity = new RigidTransform2d.Delta(0, 0, 0);
    	turretRotation = new InterpolatingTreeMap<>(kObservationBufferSize);
    	turretRotation.put(new InterpolatingDouble(start_time), initial_turret_rotation);
    	goalTrack = null;
    	nextID = 0;
    	cameraPitchCorrection = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
    	cameraYawCorrection = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
    	differentialHeight = Constants.kCenterOfTargetHeight - Constants.kCameraZOffset;
    }
    
    public static final RigidTransform2d kVehicleToTurretFixed = new RigidTransform2d(
            new Translation2d(0, 0),
            Rotation2d.fromDegrees(-90));

    public static final RigidTransform2d kTurretRotatingToCamera = new RigidTransform2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), new Rotation2d());
    
    public synchronized RigidTransform2d getFieldToVehicle(double timestamp) {
        return fieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }
    
    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
        return fieldToVehicle.lastEntry();
    }
    
    public synchronized Rotation2d getTurretRotation(double timestamp) {
        return turretRotation.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestTurretRotation() {
        return turretRotation.lastEntry();
    }
    
    public synchronized RigidTransform2d getFieldToTurretRotated(double timestamp) {
        InterpolatingDouble key = new InterpolatingDouble(timestamp);
        return fieldToVehicle.getInterpolated(key).transformBy(kVehicleToTurretFixed)
                .transformBy(RigidTransform2d.fromRotation(turretRotation.getInterpolated(key)));
    }

    public synchronized RigidTransform2d getFieldToCamera(double timestamp) {
        return getFieldToTurretRotated(timestamp).transformBy(kTurretRotatingToCamera);
    }
    
    public synchronized ShooterAimingParameters getAimingParameters(double current_timestamp) {
    	ShooterAimingParameters rv = new ShooterAimingParameters(0.0, new Rotation2d(), -1);
    	if(goalTrack != null){
        
        TrackReport report = new TrackReport(goalTrack);

        // turret fixed (latest) -> vehicle (latest) -> field
        RigidTransform2d latest_turret_fixed_to_field = getLatestFieldToVehicle().getValue()
                .transformBy(kVehicleToTurretFixed).inverse();

            if (current_timestamp - report.latest_timestamp <= kMaxTargetAge) {
            // turret fixed (latest) -> vehicle (latest) -> field -> goals
            /*RigidTransform2d latest_turret_fixed_to_goal = latest_turret_fixed_to_field
                    .transformBy(RigidTransform2d.fromTranslation(report.field_to_goal));*/
            	RigidTransform2d latest_turret_fixed_to_goal = RigidTransform2d.fromTranslation(report.field_to_goal);
            	
            	Rotation2d camAngle = new Rotation2d(latest_turret_fixed_to_goal.getTranslation().getX(),
                        latest_turret_fixed_to_goal.getTranslation().getY(), true);
            	
            	Double distance = latest_turret_fixed_to_goal.getTranslation().norm();
            	
            	Rotation2d turretAngle = new Rotation2d(Math.cos(-camAngle.getRadians())*distance + Constants.kCameraYOffset, Math.sin(-camAngle.getRadians())*distance + Constants.kCameraXOffset, true);

            // We can actually disregard the angular portion of this pose. It is
            // the bearing that we care about!
            rv = new ShooterAimingParameters(latest_turret_fixed_to_goal.getTranslation().norm(),
                    turretAngle,
                    report.id);
        }
    	}
        return rv;
    }
    
    public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        List<Translation2d> field_to_goals = new ArrayList<>();
        RigidTransform2d field_to_camera = getFieldToCamera(timestamp);
        
        if (!(vision_update == null || vision_update.isEmpty())) {
            for (TargetInfo target : vision_update) {
                double ydeadband = (target.getY() > -Constants.kCameraDeadband
                        && target.getY() < Constants.kCameraDeadband) ? 0.0 : target.getY();

                // Compensate for camera yaw
                double xyaw = target.getX() * cameraYawCorrection.cos() + ydeadband * cameraYawCorrection.sin();
                double yyaw = ydeadband * cameraYawCorrection.cos() - target.getX() * cameraYawCorrection.sin();
                double zyaw = target.getZ();

                // Compensate for camera pitch
                double xr = zyaw * cameraPitchCorrection.sin() + xyaw * cameraPitchCorrection.cos();
                double yr = yyaw;
                double zr = zyaw * cameraPitchCorrection.cos() - xyaw * cameraPitchCorrection.sin();

                // find intersection with the goal
                if (zr > 0) {
                	SmartDashboard.putNumber("Vision xr", xr);
                	SmartDashboard.putNumber("Vision yr", yr);
                    double scaling = differentialHeight / zr;
                    double distance = Math.hypot(xr, yr) * scaling;
                    Rotation2d angle = new Rotation2d(xr, yr, true);
                    /*field_to_goals.add(field_to_camera
                            .transformBy(RigidTransform2d
                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
                            .getTranslation());*/
                    Translation2d transform = RigidTransform2d.fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())).getTranslation();
                    field_to_goals.add(RigidTransform2d.fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())).getTranslation());
                    //visionAngle = Math.toDegrees(Math.atan2(Math.sin(angle.getRadians())*distance - 2.5, Math.cos(angle.getRadians())*distance + 9.5));
                    Rotation2d newAngle = new Rotation2d(Math.cos(-angle.getRadians())*distance + Constants.kCameraYOffset, Math.sin(-angle.getRadians())*distance + Constants.kCameraXOffset, true);
                    visionAngle = newAngle.getDegrees();
                    distanceToTarget = distance;
                    SmartDashboard.putNumber("Vision Angle", visionAngle);
                    SmartDashboard.putNumber("Original Vision Angle", angle.getDegrees());
                    SmartDashboard.putNumber("Vision Distance", distance);
                    SmartDashboard.putString("Vision Translation", transform.toString());
                    seesTarget = true;
                }
            }
        }else{
        	visionAngle = Double.POSITIVE_INFINITY;
        	seesTarget = false;
        }
        synchronized (this) {
        	if(field_to_goals.size() > 0){
	        	if(goalTrack == null || !goalTrack.isAlive()){
	        		goalTrack = GoalTrack.makeNewTrack(timestamp, field_to_goals.get(0), nextID);
	        		++nextID;
	        	}else{
	        		goalTrack.tryUpdate(timestamp, field_to_goals.get(0));
	        	}
        	}
        }
    }
    
    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform2d observation) {
    	fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addTurretRotationObservation(double timestamp, Rotation2d observation) {
    	turretRotation.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, RigidTransform2d field_to_vehicle,
            Rotation2d turret_rotation) {
        addFieldToVehicleObservation(timestamp, field_to_vehicle);
        addTurretRotationObservation(timestamp, turret_rotation);
    }
    public void outputToSmartDashboard(){
    	SmartDashboard.putBoolean("Vision Target Seen", seesTarget);
    }
}
