package Auto.Modes;

import java.util.Arrays;

import Auto.AutoModeBase;
import Auto.AutoModeEndedException;
import Auto.Actions.FollowPathAction;
import Auto.Actions.ParallelAction;
import Auto.Actions.PickUpGearAction;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;
import Subsystems.Swerve;
import Utilities.Util;

public class BlueGearAndHopperMode extends AutoModeBase{
	private RoboSystem robot;
	
	public BlueGearAndHopperMode(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public void routine() throws AutoModeEndedException{
		robot.pidgey.setAngle(0);
		robot.turret.resetAngle(90);
		robot.gearIntake.setState(GearIntake.State.EXTENDED_INTAKING);
		runAction(new ParallelAction(Arrays.asList(new FollowPathAction(robot.swerve.leftPegTrajectory, Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 60), 0.98), 
				new PickUpGearAction())));
		robot.gearIntake.setState(GearIntake.State.SCORING);
		runAction(new FollowPathAction(robot.swerve.leftPegToHopperTrajectory, Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 180), 0.98));
	}
}
