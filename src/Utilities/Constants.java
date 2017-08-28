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
	public static final double FRONT_RIGHT_TURN_OFFSET = 56.25;
    public static final double FRONT_LEFT_TURN_OFFSET  = 99.84;
    public static final double REAR_LEFT_TURN_OFFSET   = 278.43;
    public static final double REAR_RIGHT_TURN_OFFSET  = 355.42;
    
    public static final double SWERVE_ENCODER_REVS_PER_WHEEL_REV = 4.16458;
    public static final double SWERVE_ENCODER_REVS_PER_INCH = SWERVE_ENCODER_REVS_PER_WHEEL_REV/(Math.PI*3);
    
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
	
	//Shooter Constants
	public static final double SHOOTING_SPEED = 2650; // 2650
	public static final double SHOOTER_ALLOWABLE_ERROR = 100;
	
	//Sweeper Constants
	public static final double SWEEPER_FORWARD = 0.8;
	public static final double SWEEPER_ROLLER_FORWARD = 1.0;
    public static final double SWEEPER_ROLLER_REVERSE = -1.0;
	
    //Hanger Constants
	public static final double HANG_POWER = -1.0;
	public static final double HANG_CURRENT_THRESHOLD = 40;
	
	//Gear Intake Constants
	public static final double GEAR_DETECT_CURRENT = 12;
	public static final double GEAR_PRESENT_CURRENT = 3.0;
	public static final int CYCLES_FOR_LOST_GEAR = 7;
	public static final int CYCLES_FOR_GEAR_DETECT = 5;
	
	//Vision
	public static final int kAndroidAppTcpPort = 8254;
	public static final double kMaxTrackerDistance = 18.0;
	public static final double kMaxGoalTrackAge = 0.3;
	public static double kCameraFrameRate = 30.0;
	
	// Pose of the camera frame w.r.t. the turret frame
    public static double kCameraXOffset = -2.25;//-2.25;
    public static double kCameraYOffset = 9.875;
    public static double kCameraZOffset = 23;
    public static double kCameraPitchAngleDegrees = 90-58.5; // 35.75
    public static double kCameraYawAngleDegrees = 0.0;  //2.5 //positive moves the turret to the left
    public static double kCameraDeadband = 0.0;
    public static double kCenterOfTargetHeight = 88.0; // inches      
}