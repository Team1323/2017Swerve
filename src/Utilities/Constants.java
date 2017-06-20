package Utilities;


/**
 *
 * @author Rohi Zacharia
 */
public class Constants {
    
    public static final double WHEELBASE_LENGTH = 22.181;
    public static final double WHEELBASE_WIDTH  = 15.681;
    public static final double SWERVE_R = 27.164;
	public static final double ANGLE_FRONT_MODULE_CENTER = Math.atan(WHEELBASE_LENGTH/WHEELBASE_WIDTH);
	
    // 2017-03-05 Added little adjustments to the offsets
    public static final double FRONT_RIGHT_TURN_OFFSET = 39.375;
    public static final double FRONT_LEFT_TURN_OFFSET  = 97.73;
    public static final double REAR_LEFT_TURN_OFFSET   = 278.43;
    public static final double REAR_RIGHT_TURN_OFFSET  = 355.43;
}