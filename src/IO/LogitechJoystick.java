package IO;

import Utilities.Util;
import edu.wpi.first.wpilibj.Joystick;

public class LogitechJoystick extends Joystick{
	private static final double DEAD_BAND = 0.15;
    private boolean rumbling = false;
    
    public ButtonCheck triggerButton;
    public ButtonCheck thumbButton;
    public ButtonCheck bottomLeft;
    public ButtonCheck bottomRight;
    public ButtonCheck topLeft;
    public ButtonCheck topRight;
    
    public static final int TRIGGER_BUTTON = 1;
    public static final int THUMB_BUTTON = 2;
    public static final int BOTTOM_LEFT = 3;
    public static final int BOTTOM_RIGHT = 4;
    public static final int TOP_LEFT = 5;
    public static final int TOP_RIGHT = 6;
    
    public LogitechJoystick(int usb){
    	super(usb);
    	triggerButton = new ButtonCheck(TRIGGER_BUTTON);
    	thumbButton = new ButtonCheck(THUMB_BUTTON);
    	bottomLeft = new ButtonCheck(BOTTOM_LEFT);
    	bottomRight = new ButtonCheck(BOTTOM_RIGHT);
    	topLeft = new ButtonCheck(TOP_LEFT);
    	topRight = new ButtonCheck(TOP_RIGHT);
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
    
    public void update(){
    	triggerButton.update();
    	thumbButton.update();
    	bottomLeft.update();
    	bottomRight.update();
    	topLeft.update();
    	topRight.update();
    }
}
