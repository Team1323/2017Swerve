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
    public static final double FRONT_RIGHT_TURN_OFFSET = 268.59;
    public static final double FRONT_LEFT_TURN_OFFSET  = 97.73;
    public static final double REAR_LEFT_TURN_OFFSET   = 278.43;
    public static final double REAR_RIGHT_TURN_OFFSET  = 355.43;
    
    public static double kLooperDt = 0.01;
    
	public static final double TURRET_MAX_ANGLE = 110;
	public static final double TURRET_DEFAULT_P = 1.75;//1.75
	public static final double TURRET_DEFAULT_D = 10;//25
	
	public static final double TURRET_SMALL_P = 10.0;
	public static final double TURRET_SMALL_D = 10.0;
	public static final double TURRET_SMALL_PID_THRESH = 5;
	public static final int    TURRET_ONTARGET_THRESH  = 5;
	public static final double TURRET_TICKS_PER_REV = 7748.0;
	public static final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV/360;
	public static final double TURRET_REVS_PER_DEGREE = TURRET_TICKS_PER_DEGREE/1440;
	public static final double TURRET_ENC_REVS_PER_ACTUAL_REV = TURRET_TICKS_PER_REV/1440;
}