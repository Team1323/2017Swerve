package Auto.Actions;

import Subsystems.RoboSystem;
import Subsystems.Shooter;
import Subsystems.Turret;
import Utilities.Constants;

public class TurnOnShooterAction implements Action{
	private RoboSystem robot;
	private boolean isDone;
	
	public TurnOnShooterAction(){
		robot = RoboSystem.getInstance();
		isDone = false;
	}
	
	@Override
	public boolean isFinished(){
		return robot.shooter.onTarget();
	}
	
	@Override
	public void update(){
		/*if(robot.turret.onTarget()){
			robot.turret.gyroLock();
		}*/
	}
	
	@Override
	public void done(){
	}
	
	@Override
	public void start(){
		robot.shooter.setSpinUp(Constants.SHOOTING_SPEED);
	}
}
