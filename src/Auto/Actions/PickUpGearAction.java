package Auto.Actions;

import Subsystems.GearIntake;
import Subsystems.RoboSystem;

public class PickUpGearAction implements Action{
	private RoboSystem robot;
	private boolean isDone;
	
	public PickUpGearAction(){
		robot = RoboSystem.getInstance();
		isDone = false;
	}
	
	@Override
	public boolean isFinished(){
		return isDone;
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
	
	@Override
	public void start(){
		robot.gearIntake.setState(GearIntake.State.RETRACTED_HOLDING);
		isDone = true;
	}
}
