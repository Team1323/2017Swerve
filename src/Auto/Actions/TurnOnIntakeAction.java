package Auto.Actions;

import Subsystems.RoboSystem;

public class TurnOnIntakeAction implements Action{
	private boolean isDone = false;
	private final RoboSystem robot = RoboSystem.getInstance();
	
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
		robot.intake.intakeForward();
		isDone = true;
	}
}
