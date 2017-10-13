package Auto.Modes;

import java.util.Arrays;

import Auto.AutoModeBase;
import Auto.AutoModeEndedException;
import Auto.Actions.DeployBallFlapAction;
import Auto.Actions.DriveStraightAction;
import Auto.Actions.ExtendIntakeAction;
import Auto.Actions.ParallelAction;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;

public class MiddleGearMode extends AutoModeBase{
	private RoboSystem robot;
	
	public MiddleGearMode(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public void routine() throws AutoModeEndedException{
		robot.pidgey.setAngle(0);
		robot.turret.resetAngle(-90);
		robot.gearIntake.setState(GearIntake.State.RETRACTED_HOLDING);
		runAction(new ParallelAction(Arrays.asList(new DriveStraightAction(70, 0), 
				new ExtendIntakeAction(), new DeployBallFlapAction())));
		robot.gearIntake.score();
		runAction(new DriveStraightAction(-24, 0));
	}
}
