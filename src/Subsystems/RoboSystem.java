package Subsystems;

import java.util.ArrayList;
import java.util.List;

import Utilities.Constants;
import Utilities.Ports;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.Translation2d;
import Vision.GoalTrack;
import Vision.TargetInfo;
import edu.wpi.first.wpilibj.Solenoid;

public class RoboSystem {
	static RoboSystem instance = null;
	public Intake intake = Intake.getInstance();
	public Pidgeon pidgey = Pidgeon.getInstance();
	public Swerve swerve = Swerve.getInstance();
	public Turret turret = Turret.getInstance();
	public Hanger hanger = Hanger.getInstance();
	public GearIntake gearIntake = GearIntake.getInstance();
	public Shooter shooter = Shooter.getInstance();
	public Sweeper sweeper = Sweeper.getInstance();
	
	public Solenoid ballFlap;
	
	Rotation2d camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
	Rotation2d camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
	double differential_height_ = Constants.kCenterOfTargetHeight - Constants.kCameraZOffset;
	GoalTrack goalTrack = null;
	int nextId = 0;
	
	public static RoboSystem getInstance(){
		if(instance == null){
			instance = new RoboSystem();
		}
		return instance;
	}
	public RoboSystem(){
		ballFlap = new Solenoid(Ports.BALL_FLAP);
	}
	public void extendBallFlap(){
		ballFlap.set(true);
	}
	public void retractBallFlap(){
		ballFlap.set(false);
	}
	
	public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        List<Translation2d> field_to_goals = new ArrayList<>();
        if (!(vision_update == null || vision_update.isEmpty())) {
            for (TargetInfo target : vision_update) {
                double ydeadband = (target.getY() > -Constants.kCameraDeadband
                        && target.getY() < Constants.kCameraDeadband) ? 0.0 : target.getY();

                // Compensate for camera yaw
                double xyaw = target.getX() * camera_yaw_correction_.cos() + ydeadband * camera_yaw_correction_.sin();
                double yyaw = ydeadband * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
                double zyaw = target.getZ();

                // Compensate for camera pitch
                double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
                double yr = yyaw;
                double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();

                // find intersection with the goal
                if (zr > 0) {
                    double scaling = differential_height_ / zr;
                    double distance = Math.hypot(xr, yr) * scaling;
                    Rotation2d angle = new Rotation2d(xr, yr, true);
                    field_to_goals.add(RigidTransform2d
                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin()))
                            .getTranslation());
                }
            }
        }
        synchronized (this) {
        	if(goalTrack == null || !goalTrack.isAlive()){
        		goalTrack = GoalTrack.makeNewTrack(timestamp, field_to_goals.get(0), nextId);
        		++nextId;
        	}else{
        		goalTrack.tryUpdate(timestamp, field_to_goals.get(0));
        	}
        }
    }
}
