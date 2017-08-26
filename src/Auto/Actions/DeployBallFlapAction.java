package Auto.Actions;

import Subsystems.RoboSystem;

public class DeployBallFlapAction implements Action{
	public boolean isDone = false;
	private final RoboSystem robot;
	
	public DeployBallFlapAction(){
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
		robot.extendBallFlap();
		isDone = true;
	}
}
