package Utilities;


/**
 *
 * @author Rohi Zacharia
 */
public class Constants {
    //Swerve Calculations Constants
    public static final double WHEELBASE_LENGTH = 22.181;
    public static final double WHEELBASE_WIDTH  = 15.681;
    public static final double SWERVE_R = 27.164;
	public static final double ANGLE_FRONT_MODULE_CENTER = Math.atan(WHEELBASE_LENGTH/WHEELBASE_WIDTH);
	
    // Swerve Module Wheel Offsets
    public static final double FRONT_RIGHT_TURN_OFFSET = 259.8;
    public static final double FRONT_LEFT_TURN_OFFSET  = 97.73;
    public static final double REAR_LEFT_TURN_OFFSET   = 278.43;
    public static final double REAR_RIGHT_TURN_OFFSET  = 355.43;
    
    public static double kLooperDt = 0.01;
    
    public static final double TURRET_CLICKS_TO_ANGLE = 66.2252;
	public static final int    TURRET_TICKS_PER_90    = 22;
	public static final double TURRET_MAX_ANGLE = 110;
	public static final double TURRET_DEFAULT_P = 1.75;//1.75
	public static final double TURRET_DEFAULT_D = 10;//25
	
	public static final double TURRET_SMALL_P = 10.0;
	public static final double TURRET_SMALL_D = 10.0;
	public static final double TURRET_SMALL_PID_THRESH = 5;
	public static final int    TURRET_ONTARGET_THRESH  = 5;
	public static final double STICK_DEAD_BAND = 0.0;
	
	public static final double 
}