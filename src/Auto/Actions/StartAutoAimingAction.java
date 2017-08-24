package Auto.Actions;

import Subsystems.Turret;

public class StartAutoAimingAction implements Action{
	private boolean isDone;
	private final Turret turret = Turret.getInstance();
	
	public StartAutoAimingAction(){
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
		turret.enableVision();
		isDone = true;
	}
}
