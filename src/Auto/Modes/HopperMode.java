package Auto.Modes;

import java.util.Arrays;

import Auto.AutoModeBase;
import Auto.AutoModeEndedException;
import Auto.Actions.DeployBallFlapAction;
import Auto.Actions.ExtendIntakeAction;
import Auto.Actions.FollowPathAction;
import Auto.Actions.ParallelAction;
import Auto.Actions.ReciprocateBallFlapAction;
import Auto.Actions.SeriesAction;
import Auto.Actions.StartAutoAimingAction;
import Auto.Actions.TurnOnShooterAction;
import Auto.Actions.TurnOnSweeperAction;
import Auto.Actions.WaitForAutoAimAction;
import Subsystems.RoboSystem;
import Subsystems.Swerve;

public class HopperMode extends AutoModeBase{
	private Swerve.Path path;
	private RoboSystem robot;
	private double turretAngle;
	private int pigeonAngle;
	
	public HopperMode(Swerve.Path path, double turretAngle, int pigeonAngle){
		this.path = path;
		this.turretAngle = turretAngle;
		this.pigeonAngle = pigeonAngle;
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public void routine() throws AutoModeEndedException{
		robot.pidgey.setAngle(pigeonAngle);
		robot.turret.resetAngle(turretAngle);
		
		runAction(new ParallelAction(Arrays.asList(new FollowPathAction(path), new ExtendIntakeAction(), 
				new DeployBallFlapAction(), new StartAutoAimingAction())));
		
		WaitForAutoAimAction autoAim = new WaitForAutoAimAction();
		runAction(autoAim);
		if(autoAim.timedOut){
			System.out.println("Auto Aim timed out");
			if(turretAngle == 90){
				robot.turret.gyroLock(-pigeonAngle, 100);
			}else if(turretAngle == -90){
				robot.turret.gyroLock(-pigeonAngle, -100);
			}
		}
		
		runAction(new SeriesAction(Arrays.asList(new TurnOnShooterAction(), 
				new TurnOnSweeperAction(), new ReciprocateBallFlapAction(1, 3)))); 
	}
	
}
