package Auto.Actions;

import Subsystems.Turret;

public class StartAutoAimingAction implements Action{
	private boolean isDone;
	private final Turret turret;
	
	public StartAutoAimingAction(){
		isDone = false;
		turret = Turret.getInstance();
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
		turret.enableVision();
		isDone = true;
	}
}
