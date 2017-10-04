package Subsystems;


import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

import Utilities.Constants;
import Utilities.InterpolatingDouble;
import Utilities.InterpolatingTreeMap;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.ShooterAimingParameters;
import Utilities.Translation2d;
import Vision.GoalTracker;
import Vision.GoalTracker.TrackReport;
import Vision.TargetInfo;
import edu.wpi.first.wpilibj.Timer;
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
    Map<Double, Double> mObservedAngles = new TreeMap<>();
    double smoothedAngle;
    public double getSmoothedVisionAngle(){
    	return smoothedAngle;
    }
    private GoalTracker goalTracker;
    protected Rotation2d cameraPitchCorrection;
    protected Rotation2d cameraYawCorrection;
    protected double differentialHeight;
    double visionAngle = 0.0;
    double distanceToTarget = 0.0;
    public double getVisionAngle(){
    	return visionAngle;
    }
    double originalVisionAngle = 0.0;
    public double getOriginalVisionAngle(){
    	return originalVisionAngle;
    }
    boolean seesTarget = false;
    public boolean getTargetVisbility(){
    	return seesTarget;
    }
    public double getTargetDistance(){
    	return distanceToTarget;
    }
    
    protected RobotState() {
        reset(0, RigidTransform2d.fromRotation(Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(-90));
    }

    public synchronized void reset(double start_time, RigidTransform2d initial_field_to_vehicle,
            Rotation2d initial_turret_rotation) {
    	fieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
    	fieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
    	vehicleVelocity = new RigidTransform2d.Delta(0, 0, 0);
    	turretRotation = new InterpolatingTreeMap<>(kObservationBufferSize);
    	turretRotation.put(new InterpolatingDouble(start_time), initial_turret_rotation);
    	goalTracker = new GoalTracker();
    	cameraPitchCorrection = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
    	cameraYawCorrection = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
    	differentialHeight = Constants.kCenterOfTargetHeight - Constants.kCameraZOffset;
    }
    
    public static final RigidTransform2d kVehicleToTurretFixed = new RigidTransform2d(
            new Translation2d(0, 0),
            Rotation2d.fromDegrees(90));

    public static final RigidTransform2d kTurretRotatingToCamera = new RigidTransform2d(
            new Translation2d(Constants.kCameraYOffset, -Constants.kCameraXOffset), new Rotation2d());
    
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
    
    public synchronized Optional<ShooterAimingParameters> getAimingParameters(double current_timestamp) {
    	List<TrackReport> reports = goalTracker.getTracks();
    	if(!reports.isEmpty()){
    		TrackReport report = reports.get(0);
    		RigidTransform2d latest_turret_fixed_to_goal = getLatestFieldToVehicle().getValue()
    				.transformBy(kVehicleToTurretFixed).inverse()
    				.transformBy(RigidTransform2d.fromTranslation(report.field_to_goal));
    		SmartDashboard.putString("Boiler Position", "(" + report.field_to_goal.getX() + ", " + report.field_to_goal.getY() + ")");
    		ShooterAimingParameters params = new ShooterAimingParameters(latest_turret_fixed_to_goal.getTranslation().norm(),
    				new Rotation2d(latest_turret_fixed_to_goal.getTranslation().getX(), latest_turret_fixed_to_goal.getTranslation().getY(), true),
    				report.id);
    		
    		return Optional.of(params);
    	}else{
    		return Optional.empty();
    	}
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
                    Rotation2d newAngle = new Rotation2d(Math.cos(-angle.getRadians())*distance + Constants.kCameraYOffset, Math.sin(-angle.getRadians())*distance + Constants.kCameraXOffset, true);
                    visionAngle = newAngle.getDegrees();
                    distanceToTarget = distance;
                    originalVisionAngle = angle.getDegrees();
                    mObservedAngles.put(timestamp, visionAngle);
                    field_to_goals.add(field_to_camera
                            .transformBy(RigidTransform2d
                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
                            .getTranslation());
                    pruneByTime();
                    SmartDashboard.putNumber("Vision Angle", visionAngle);
                    SmartDashboard.putNumber("Original Vision Angle", angle.getDegrees());
                    SmartDashboard.putNumber("Vision Distance", distance);
                    SmartDashboard.putNumber("Vision Smoothed Angle", smoothedAngle);
                    seesTarget = true;
                }
            }
        }else{
        	visionAngle = Double.POSITIVE_INFINITY;
        	seesTarget = false;
        	pruneByTime();
        }
        synchronized (this) {
        	goalTracker.update(timestamp, field_to_goals);
        }
    }
    
    void pruneByTime() {
        double delete_before = Timer.getFPGATimestamp() - Constants.kMaxAngleAge;
        for (Iterator<Map.Entry<Double, Double>> it = mObservedAngles.entrySet().iterator(); it.hasNext();) {
            Map.Entry<Double, Double> entry = it.next();
            if (entry.getKey() < delete_before) {
                it.remove();
            }
        }
        if (mObservedAngles.isEmpty()) {
            smoothedAngle = Double.POSITIVE_INFINITY;
        } else {
            smooth();
        }
    }
    
    void smooth() {
        if (mObservedAngles.size() > 0) {
            double x = 0;
            for (Map.Entry<Double, Double> entry : mObservedAngles.entrySet()) {
                x += entry.getValue();
            }
            x /= mObservedAngles.size();
            smoothedAngle = x;
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
    	double time = Timer.getFPGATimestamp();
    	RigidTransform2d robotPose = getLatestFieldToVehicle().getValue();
    	RigidTransform2d turretPose = getFieldToTurretRotated(time);
    	RigidTransform2d cameraPose = getFieldToCamera(time);
    	SmartDashboard.putBoolean("Vision Target Seen", seesTarget);
    	if(getAimingParameters(time).isPresent()){
    		SmartDashboard.putNumber("Turret To Goal Angle", getAimingParameters(Timer.getFPGATimestamp()).get().getTurretAngle().getDegrees());
    	}
    	SmartDashboard.putString("Turret Field Pose", "(" + turretPose.getTranslation().getX() + ", " +
    	    	turretPose.getTranslation().getY() + ", " + turretPose.getRotation().getDegrees());
    	SmartDashboard.putString("Field To Vehicle Pose", "(" + robotPose.getTranslation().getX() + ", " +
    			robotPose.getTranslation().getY() + ", " + robotPose.getRotation().getDegrees());
    	SmartDashboard.putString("Field to Camera Pose", "(" + cameraPose.getTranslation().getX() + ", " +
    	    	cameraPose.getTranslation().getY() + ", " + cameraPose.getRotation().getDegrees());
    }
}
