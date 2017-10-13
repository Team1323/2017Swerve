package Auto.Actions;

import Subsystems.RoboSystem;
import Subsystems.Swerve;

public class DriveStraightAction implements Action{
	private RoboSystem robot;
	public double distance;
	public double wheelHeading;
	private boolean hasStarted;
	
	public DriveStraightAction(double distance, double wheelHeading){
		hasStarted = false;
		this.distance = distance;
		this.wheelHeading = wheelHeading;
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return hasStarted && robot.swerve.distanceOnTarget();
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		robot.swerve.setState(Swerve.ControlState.Neutral);
	}
	
	@Override
	public void start(){
		robot.swerve.moveDistance(wheelHeading, distance);
		hasStarted = true;
	}
}
