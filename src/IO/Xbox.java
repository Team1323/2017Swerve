package IO;

import java.util.Timer;
import java.util.TimerTask;

import Utilities.Util;
import edu.wpi.first.wpilibj.XboxController;

public class Xbox extends XboxController{
    private static final double PRESS_THRESHOLD = 0.3;
    private static final double DEAD_BAND = 0.15;
    private final Timer mTimer = new Timer();
    private static final int K_READING_RATE = 10;
    private boolean rumbling = false;
    public ButtonCheck aButton;
    public ButtonCheck bButton;
    public ButtonCheck xButton;
    public ButtonCheck yButton;
    public ButtonCheck startButton;
    public ButtonCheck backButton;
    public ButtonCheck leftBumper;
    public ButtonCheck rightBumper;
    public ButtonCheck leftCenterClick;
    public ButtonCheck rightCenterClick;
    private RumbleThread rumbler;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
    public static final int LEFT_CENTER_CLICK = 9;
    public static final int RIGHT_CENTER_CLICK = 10;
    
    public Xbox(int usb)   { 
    	super(usb);
   }
    public void init(){
    	aButton = new ButtonCheck(A_BUTTON);
        bButton = new ButtonCheck(B_BUTTON);
        xButton = new ButtonCheck(X_BUTTON);
        yButton = new ButtonCheck(Y_BUTTON);
        startButton = new ButtonCheck(START_BUTTON);
        backButton = new ButtonCheck(BACK_BUTTON);
        leftBumper = new ButtonCheck(LEFT_BUMPER);
        rightBumper = new ButtonCheck(RIGHT_BUMPER);
        leftCenterClick = new ButtonCheck(LEFT_CENTER_CLICK);
        rightCenterClick = new ButtonCheck(RIGHT_CENTER_CLICK);        
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
    public enum Vibration{
    	SINGLE, DOUBLE,PULSE
    }
    public void rumble(Vibration _mode){
    	if(!rumbling){
	    	rumbler = new RumbleThread();
	    	rumbler.setVibration(_mode);
	    	rumbler.start();
    	}
    }
    public class RumbleThread extends Thread{
    	private Vibration mode;
		@Override
		public void run() {
			rumbling = true;
			switch(mode){
	    	case SINGLE:	    		
	    		try {	    			
	    			setRumble(RumbleType.kLeftRumble,1);
		    		setRumble(RumbleType.kRightRumble,1);
					sleep(500);
					setRumble(RumbleType.kLeftRumble,0);
		    		setRumble(RumbleType.kRightRumble,0);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					rumbling = false;
					e.printStackTrace();
				}
	    		break;
	    	case DOUBLE:
	    		try {
	    			setRumble(RumbleType.kLeftRumble,1);
		    		setRumble(RumbleType.kRightRumble,1);
					sleep(250);
					setRumble(RumbleType.kLeftRumble,0);
		    		setRumble(RumbleType.kRightRumble,0);
		    		sleep(250);
		    		setRumble(RumbleType.kLeftRumble,1);
		    		setRumble(RumbleType.kRightRumble,1);
					sleep(250);
					setRumble(RumbleType.kLeftRumble,0);
		    		setRumble(RumbleType.kRightRumble,0);
		    		sleep(500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
					rumbling = false;
				}
	    		break;
	    	case PULSE:
	    		
	    		break;
	    	}
			rumbling = false;
		}
		public void setVibration(Vibration _mode){
	    	mode = _mode;
	    }
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
    private class UpdateTask extends TimerTask {
	    public void run(){ 
	    	aButton.update();
	    	bButton.update();
	    	xButton.update();
	    	yButton.update();
	    	startButton.update();
	    	backButton.update();
	    	leftBumper.update();
	    	rightBumper.update();
	    	leftCenterClick.update();
	    	rightCenterClick.update();
	    }
    }
}