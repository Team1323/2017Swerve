package Auto.Modes;

import java.util.Arrays;

import Auto.AutoModeBase;
import Auto.AutoModeEndedException;
import Auto.Actions.AlignForShootingAction;
import Auto.Actions.DeployBallFlapAction;
import Auto.Actions.ExtendIntakeAction;
import Auto.Actions.FollowPathAction;
import Auto.Actions.ParallelAction;
import Auto.Actions.ReciprocateBallFlapAction;
import Auto.Actions.SeriesAction;
import Auto.Actions.StartAutoAimingAction;
import Auto.Actions.TurnOnShooterAction;
import Auto.Actions.TurnOnSweeperAction;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;
import jaci.pathfinder.Trajectory;

public class HopperMode extends AutoModeBase{
	private Trajectory trajectory;
	private RoboSystem robot;
	private double turretAngle;
	private int pigeonAngle;
	
	public HopperMode(Trajectory trajectory, double turretAngle, int pigeonAngle){
		this.trajectory = trajectory;
		this.turretAngle = turretAngle;
		this.pigeonAngle = pigeonAngle;
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public void routine() throws AutoModeEndedException{
		robot.pidgey.setAngle(pigeonAngle);
		robot.turret.resetAngle(turretAngle);
		robot.gearIntake.setState(GearIntake.State.RETRACTED_HOLDING);
		System.out.println("reached");
		runAction(new ParallelAction(Arrays.asList(new FollowPathAction(trajectory, -pigeonAngle, 0.95), new ExtendIntakeAction(), 
				new DeployBallFlapAction(), new StartAutoAimingAction())));
		runAction(new TurnOnShooterAction());
		
		AlignForShootingAction alignAction = new AlignForShootingAction();
		runAction(alignAction);
		if(alignAction.noVision){
			if(turretAngle == 90){
				robot.turret.setGyroLockAngle(-pigeonAngle, 96);
			}else{
				robot.turret.setGyroLockAngle(-pigeonAngle, -100);
			}
		}
		runAction(new SeriesAction(Arrays.asList(/*new WaitForAutoAimAction(turretAngle, pigeonAngle),*/ 
				new TurnOnSweeperAction(), new ReciprocateBallFlapAction(1.5, 3)))); 
	}
	
}
