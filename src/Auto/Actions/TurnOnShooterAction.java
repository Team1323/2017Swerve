package Auto.Actions;

import Subsystems.RoboSystem;
import Subsystems.Shooter;
import Subsystems.Turret;
import Utilities.Constants;

public class TurnOnShooterAction implements Action{
	private RoboSystem robot;
	
	public TurnOnShooterAction(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return robot.shooter.onTarget();
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){
	}
	
	@Override
	public void start(){
		robot.turret.gyroLock();
		robot.shooter.setSpeed(Constants.SHOOTING_SPEED);
	}
}
