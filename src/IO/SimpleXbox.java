package IO;

import Utilities.Util;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class SimpleXbox extends XboxController{
	private static final double PRESS_THRESHOLD = 0.3;
    private static final double DEAD_BAND = 0.15;
    
    public SimpleXbox(int usb)   { 
    	super(usb);
   }
    
    @Override
    public double getX(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
          return Util.deadBand(getRawAxis(0), DEAD_BAND);
        } else {
          return Util.deadBand(getRawAxis(4), DEAD_BAND);
        }
      }
    @Override
    public double getY(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
          return Util.deadBand(getRawAxis(1), DEAD_BAND);
        } else {
          return Util.deadBand(getRawAxis(5), DEAD_BAND);
        }
      }
    @Override
    public double getTriggerAxis(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
          return Util.deadBand(getRawAxis(2), PRESS_THRESHOLD);
        } else {
          return Util.deadBand(getRawAxis(3), PRESS_THRESHOLD);
        }
      }
}
