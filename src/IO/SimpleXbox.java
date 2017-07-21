package IO;

import Utilities.Util;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class SimpleXbox extends XboxController{
	private static final double PRESS_THRESHOLD = 0.3;
    private static final double DEAD_BAND = 0.15;
    
    private boolean rumbling = false;
    
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
    public void rumble(double rumblesPerSecond){
    	if(!rumbling){
    		RumbleThread r = new RumbleThread(rumblesPerSecond);
    		r.start();
    	}
    }
    public class RumbleThread extends Thread{
    	public double rumblesPerSec = 1;
    	public long interval = 500;
    	public RumbleThread(double rumblesPerSecond){
    		rumblesPerSec = rumblesPerSecond;
    		interval =(long) (1/(rumblesPerSec*2)*1000);
    	}
    	public void run(){
    		rumbling = true;
    		try{
    			for(int i=0;i<rumblesPerSec;i++){
		    		setRumble(RumbleType.kLeftRumble, 1);
		    		setRumble(RumbleType.kRightRumble, 1);
		    		sleep(interval);
		    		setRumble(RumbleType.kLeftRumble, 0);
		    		setRumble(RumbleType.kRightRumble, 0);
		    		sleep(interval);
    			}
    		}catch (InterruptedException e) {
				rumbling = false;
				e.printStackTrace();
			}
    		rumbling = false;
    	}
    }
}
