package Auto.Modes;

import java.util.Arrays;

import Auto.AutoModeBase;
import Auto.AutoModeEndedException;
import Auto.Actions.AlignForShootingAction;
import Auto.Actions.FollowPathAction;
import Auto.Actions.ParallelAction;
import Auto.Actions.PickUpGearAction;
import Auto.Actions.ReciprocateBallFlapAction;
import Auto.Actions.SeriesAction;
import Auto.Actions.TurnOnShooterAction;
import Auto.Actions.TurnOnSweeperAction;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;
import Utilities.Util;

public class RedGearAndHopperMode extends AutoModeBase{
private RoboSystem robot;
	
	public RedGearAndHopperMode(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public void routine() throws AutoModeEndedException{
		robot.pidgey.setAngle(0);
		robot.turret.resetAngle(-90);
		robot.gearIntake.setState(GearIntake.State.EXTENDED_INTAKING);
		runAction(new ParallelAction(Arrays.asList(new FollowPathAction(robot.swerve.rightPegTrajectory, Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), -60), 0.98), 
				new PickUpGearAction())));
		robot.gearIntake.setState(GearIntake.State.SCORING);
		runAction(new ParallelAction(Arrays.asList(new FollowPathAction(robot.swerve.rightPegToHopperTrajectory, Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 0), 0.92),
				 new TurnOnShooterAction())));
		runAction(new SeriesAction(Arrays.asList(new AlignForShootingAction(),/*new TurnOnShooterAction(),*/ 
				new TurnOnSweeperAction(), new ReciprocateBallFlapAction(1.0, 5))));
	}
}
