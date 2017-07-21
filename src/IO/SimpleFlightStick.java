package IO;

import Utilities.Util;
import edu.wpi.first.wpilibj.Joystick;

public class SimpleFlightStick extends Joystick{
	public static final int TRIGGER_BUTTON = 1;
	public static final int LEFT_BUTTON = 3;
	public static final int RIGHT_BUTTON = 2;
	public static final int DOWN_BUTTON = 4;
	
	public static final double DEAD_BAND = 0.15;
	
	public SimpleFlightStick(int usb){
		super(usb);
	}
	
	public boolean getTriggerButton(){
		return getRawButton(TRIGGER_BUTTON);
	}
	public boolean getLeftButton(){
		return getRawButton(LEFT_BUTTON);
	}
	public boolean getRightButton(){
		return getRawButton(RIGHT_BUTTON);
	}
	public boolean getDownButton(){
		return getRawButton(DOWN_BUTTON);
	}
	public double getXAxis(){
		return Util.deadBand(getRawAxis(0), DEAD_BAND);
	}
	public double getYAxis(){
		return Util.deadBand(getRawAxis(1), DEAD_BAND);
	}
}
