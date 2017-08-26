package Auto.Actions;

import Subsystems.RoboSystem;

public class TurnOnSweeperAction implements Action{
	private boolean isDone;
	private RoboSystem robot;
	
	public TurnOnSweeperAction(){
		isDone = false;
		robot = RoboSystem.getInstance();
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
		robot.sweeper.startSweeper();
	}
	
}
