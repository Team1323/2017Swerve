package Auto.Actions;

import Subsystems.GearIntake;
import Subsystems.RoboSystem;

public class PickUpGearAction implements Action{
	private RoboSystem robot;
	
	public PickUpGearAction(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return robot.gearIntake.getState() != GearIntake.State.EXTENDED_INTAKING && robot.gearIntake.hasGear();
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
	
	@Override
	public void start(){
		robot.gearIntake.setState(GearIntake.State.EXTENDED_INTAKING);
	}
}
