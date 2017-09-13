package Auto;

import org.json.simple.JSONArray;

import Auto.Modes.HopperMode;
import Auto.Modes.StandStillMode;
import Subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardInteractions {
	private static final String AUTO_OPTIONS = "auto_options";
    private static final String SELECTED_AUTO_MODE = "selected_auto_mode";
    private static final String AUTO_SIDE = "auto_side";
    private static final String SELECTED_AUTO_SIDE = "selected_auto_side";
    
    private static final AutoOption DEFAULT_MODE = AutoOption.HOPPER;
    private static final AutoSide DEFAULT_SIDE = AutoSide.BLUE;
    
    private SendableChooser modeChooser;
    private SendableChooser sideChooser;
    
    public void initWithDefaults(){
    	modeChooser = new SendableChooser();
    	modeChooser.addDefault("Hopper", DEFAULT_MODE);
    	sideChooser = new SendableChooser();
    	sideChooser.addDefault("Blue", DEFAULT_SIDE);
    	sideChooser.addObject("Red", AutoSide.RED);
    	SmartDashboard.putData("Mode Chooser", modeChooser);
    	SmartDashboard.putData("Side Chooser", sideChooser);
    	SmartDashboard.putString(SELECTED_AUTO_MODE, DEFAULT_MODE.name);
    	SmartDashboard.putString(SELECTED_AUTO_SIDE, DEFAULT_SIDE.color);
    }
    
    public AutoModeBase getSelectedAutoMode(){
        AutoOption selectedOption =  (AutoOption)  modeChooser.getSelected();
        
        AutoSide selectedSide = (AutoSide) sideChooser.getSelected();
        
        return createAutoMode(selectedOption, selectedSide);
    }
    
    public String getSelectedSide(){
    	AutoSide side = (AutoSide) sideChooser.getSelected();
    	return side.color;
    }
    public String getSelectedMode(){
    	AutoOption option = (AutoOption) modeChooser.getSelected();
    	return option.name;
    }
    
    enum AutoOption{
    	HOPPER("Hopper"), 
    	STAND_STILL("Stand Still");
    	
    	public final String name;
    	
    	AutoOption(String name){
    		this.name = name;
    	}
    }
    
    public enum AutoSide{
    	BLUE("Blue"),RED("Red");
    	
    	public final String color;
    	
    	AutoSide(String color){
    		this.color = color;
    	}
    }
    
    private AutoModeBase createAutoMode(AutoOption option, AutoSide side){
    	switch(option){
    		case HOPPER:
    			if(side == AutoSide.BLUE){
    				return new HopperMode(Swerve.Path.BLUE_HOPPER, 90, 180);
    			}else{
    				return new HopperMode(Swerve.Path.RED_HOPPER, -90, 0);
    			}
    		case STAND_STILL: // fallthrough
            default:
                System.out.println("ERROR: unexpected auto mode: " + option);
                return new StandStillMode();
    	}
    }
    
    public void output(){
    	SmartDashboard.putString(SELECTED_AUTO_MODE, getSelectedMode());
    	SmartDashboard.putString(SELECTED_AUTO_SIDE, getSelectedSide());
    }
}
