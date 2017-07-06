package IO;

import java.util.Timer;
import java.util.TimerTask;

import Utilities.Util;
import edu.wpi.first.wpilibj.Joystick;

public class FlightStick extends Joystick{
	public ButtonCheck triggerButton;
	public ButtonCheck leftButton;
	public ButtonCheck rightButton;
	public ButtonCheck downButton;
	
	public static final int TRIGGER_BUTTON = 1;
	public static final int LEFT_BUTTON = 2;
	public static final int RIGHT_BUTTON = 3;
	public static final int DOWN_BUTTON = 4;
	
	public static final double DEAD_BAND = 0.15;
	
	private final Timer mTimer = new Timer();
	
	public FlightStick(int usb){
		super(usb);
	}
	
	public void init(){
    	triggerButton = new ButtonCheck(TRIGGER_BUTTON);    
    	leftButton = new ButtonCheck(LEFT_BUTTON);
    	rightButton = new ButtonCheck(RIGHT_BUTTON);
    	downButton = new ButtonCheck(DOWN_BUTTON);
    }
	
	public void start() {
        synchronized (mTimer) {
            mTimer.schedule(new InitTask(), 0);
        }
    }
	
	private class InitTask extends TimerTask {
        @Override
        public void run() {
            while (true) {
                try {
                	init();
                    break;
                } catch (Exception e) {
                    System.out.println("FSM failed to initialize: " + e.getMessage());
                    synchronized (mTimer) {
                        mTimer.schedule(new InitTask(), 500);
                    }
                }
            }
            synchronized (mTimer) {
                mTimer.schedule(new UpdateTask(), 0, 10);
            }
        }
    }
	
	public double getXAxis(){
		return Util.deadBand(getRawAxis(0), DEAD_BAND);
	}
	public double getYAxis(){
		return Util.deadBand(getRawAxis(1), DEAD_BAND);
	}
	
	public class ButtonCheck{
    	boolean buttonCheck = false;
    	boolean buttonActive = false;
    	boolean longPressActive = false;
    	boolean hasBeenPressed = false;
    	private double buttonStartTime = 0;
    	private int buttonNumber;
    	
    	public ButtonCheck(int id){
    		buttonNumber = id;
    	}
    	public void update(){
    		buttonCheck = getRawButton(buttonNumber);
    		if(buttonCheck){
	    		if(buttonActive){
	    			if(System.currentTimeMillis() - buttonStartTime > 250){
	    				longPressActive = true;
	    			}
	    		}else{
	    			buttonActive = true;
	    			buttonStartTime = System.currentTimeMillis();
	    		}
    		}else{
    			if(buttonActive){
    				buttonActive = false;
    				if(longPressActive){
    					hasBeenPressed = false;
    					longPressActive = false;
    				}else{
    					hasBeenPressed = true;
    				}
    			}
    		}
    	}
    	public boolean wasPressed(){
    		if(hasBeenPressed){
    			hasBeenPressed = false;
    			return true;
    		}
    		return false;
    	}
    	public boolean longPressed(){
    		return longPressActive;
    	}
    	public boolean isBeingPressed(){
    		return buttonActive;
    	}
    }
	private class UpdateTask extends TimerTask{
		public void run(){
			triggerButton.update();
			leftButton.update();
			rightButton.update();
			downButton.update();
		}
	}
}
