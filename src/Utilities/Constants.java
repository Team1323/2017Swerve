package Utilities;


/**
 *
 * @author Rohi Zacharia
 */
public class Constants {
    //Swerve Calculations Constants
    public static final double WHEELBASE_LENGTH = 22.181;
    public static final double WHEELBASE_WIDTH  = 15.681;
    public static final double WHEELBASE_HALF_LENGTH = WHEELBASE_LENGTH/2;
    public static final double WHEELBASE_HALF_WIDTH  = WHEELBASE_WIDTH/2;
    public static final double SWERVE_R = 27.164;
	public static final double ANGLE_FRONT_MODULE_CENTER = Math.atan(WHEELBASE_LENGTH/WHEELBASE_WIDTH);
	
    // Swerve Module Wheel Offsets
	public static final double FRONT_RIGHT_TURN_OFFSET = 55.5468;
    public static final double FRONT_LEFT_TURN_OFFSET  = 98.789;
    public static final double REAR_LEFT_TURN_OFFSET   = 278.0859;
    public static final double REAR_RIGHT_TURN_OFFSET  = 355.78125;
    
    //rpm 3844 4033 3900 4179 3989
    //native units 9226 9680 9362 10030 9574
    //f gain 0.10685189053687069145602673908502
    
    //module rpm 126140
    //native units 600
    //f gain 1.705
    
    public static final double SWERVE_DRIVE_MAX_RPM = 3989.0;
    
    public static final double SWERVE_WHEEL_DIAMETER = 2.8;
    public static final double SWERVE_ENCODER_REVS_PER_WHEEL_REV = 4.307692308;
    public static final double SWERVE_ENCODER_REVS_PER_INCH = SWERVE_ENCODER_REVS_PER_WHEEL_REV/(Math.PI*SWERVE_WHEEL_DIAMETER);
    
    public static double kSegmentCompletionTolerance = 0.1; // inches
    public static double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static double kPathFollowingMaxVel = 120.0; // inches per second
    public static double kPathFollowingProfileKp = 5.00;
    public static double kPathFollowingProfileKi = 0.03;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.05;
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;
    
 // Path following constants
    public static double kMinLookAhead = 6.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 12.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;
    public static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
    // our speed
    // in inches per sec
    
    //Looper Rate
    public static final double kLooperDt = 0.01;
    
    //Turret Constants
	public static final double TURRET_MAX_ANGLE = 110;
	public static final double TURRET_DEFAULT_P = 1.75;
	public static final double TURRET_DEFAULT_D = 10;
	public static final double TURRET_SMALL_P = 10.0;
	public static final double TURRET_SMALL_D = 10.0;
	public static final double TURRET_SMALL_PID_THRESH = 5;
	public static final int    TURRET_ONTARGET_THRESH  = 5;
	public static final double TURRET_TICKS_PER_REV = 7748.0;
	public static final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV/360;
	public static final double TURRET_REVS_PER_DEGREE = TURRET_TICKS_PER_DEGREE/1440;
	public static final double TURRET_ENC_REVS_PER_ACTUAL_REV = TURRET_TICKS_PER_REV/1440;
	//rpm 500
	//native units 1250
	
	//Shooter Constants
	public static final double SHOOTING_SPEED = 2675; // 2650
	public static final double SHOOTER_ALLOWABLE_ERROR = 50;//100
	
	//Sweeper Constants
	public static final double SWEEPER_FORWARD = 0.7;
	public static final double SWEEPER_ROLLER_FORWARD = -1.0;
    public static final double SWEEPER_ROLLER_REVERSE = 1.0;
	
    //Hanger Constants
	public static final double HANG_POWER = -1.0;
	public static final double HANG_STOP_CURRENT = 40;
	public static final double HANGING_DETECT_CURRENT = 15;
	
	//Gear Intake Constants
	public static final double GEAR_DETECT_CURRENT = 9;
	public static final double GEAR_PRESENT_CURRENT = 3.0;
	public static final int CYCLES_FOR_LOST_GEAR = 10;
	public static final int CYCLES_FOR_GEAR_DETECT = 3;
	
	//Vision
	public static final int kAndroidAppTcpPort = 8254;
	public static final double kMaxTrackerDistance = 18.0;
	public static final double kMaxGoalTrackAge = 1.0;
	public static final double kMaxAngleAge = 0.5;
	public static double kCameraFrameRate = 30.0;
	public static final double kOptimalShootingDistance = 84.8;
	
	// Pose of the camera frame w.r.t. the turret frame
    public static final double kCameraXOffset = -0.5;//-2.25;
    public static final double kCameraYOffset = 9.875;
    public static final double kCameraZOffset = 23;
    public static final double kCameraPitchAngleDegrees = 90-58.5; // 35.75
    public static final double kCameraYawAngleDegrees = 0.0;
    public static final double kCameraDeadband = 0.0;
    public static final double kCenterOfTargetHeight = 88.0; // inches      
    public static final double kBoilerRadius = 7.5;
}