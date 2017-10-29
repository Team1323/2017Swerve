package Auto.Modes;

import java.util.Arrays;

import Auto.AutoModeBase;
import Auto.AutoModeEndedException;
import Auto.Actions.AlignForShootingAction;
import Auto.Actions.DeployBallFlapAction;
import Auto.Actions.ExtendIntakeAction;
import Auto.Actions.FollowPathAction;
import Auto.Actions.ParallelAction;
import Auto.Actions.SeriesAction;
import Auto.Actions.TurnOnShooterAction;
import Auto.Actions.TurnOnSweeperAction;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;

public class BlueMiddleGearMode extends AutoModeBase{
	private RoboSystem robot;
	
	public BlueMiddleGearMode(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public void routine() throws AutoModeEndedException{
		robot.pidgey.setAngle(0);
		robot.turret.resetAngle(-90);
		robot.gearIntake.setState(GearIntake.State.RETRACTED_HOLDING);
		runAction(new ParallelAction(Arrays.asList(new FollowPathAction(robot.swerve.middleGearTrajectory), 
				new ExtendIntakeAction(), new DeployBallFlapAction())));
		robot.gearIntake.score();
		robot.turret.setMotionMagic(-45);
		runAction(new SeriesAction(Arrays.asList(new FollowPathAction(robot.swerve.middleToBlueBoilerTrajectory, 0, 0.95), 
				new AlignForShootingAction(), new TurnOnShooterAction(), new TurnOnSweeperAction())));
	}
}
