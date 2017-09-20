package Auto.Modes;

import java.util.Arrays;

import Auto.AutoModeBase;
import Auto.AutoModeEndedException;
import Auto.Actions.DeployBallFlapAction;
import Auto.Actions.ExtendIntakeAction;
import Auto.Actions.FollowPathAction;
import Auto.Actions.ParallelAction;
import Auto.Actions.PickUpGearAction;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;
import Subsystems.Swerve;

public class GearMode extends AutoModeBase{
	private RoboSystem robot;
	
	public GearMode(){
		robot = RoboSystem.getInstance();
	}
	
	@Override 
	public void routine() throws AutoModeEndedException{
		robot.pidgey.setAngle(0);
		robot.turret.resetAngle(-90);
		robot.gearIntake.setState(GearIntake.State.RETRACTED_OFF);
		
		runAction(new ParallelAction(Arrays.asList(new FollowPathAction(Swerve.Path.FORWARD, 0), new ExtendIntakeAction(), 
				new DeployBallFlapAction())));
		
		runAction(new FollowPathAction(Swerve.Path.BACKWARD, 0));
		
		
		
		runAction(new ParallelAction(Arrays.asList(new PickUpGearAction(), new FollowPathAction(Swerve.Path.GEAR, -7))));
		
		robot.gearIntake.score();
		
		runAction(new FollowPathAction(Swerve.Path.BACKWARD, 0));
	}
}
