package Auto.Actions;

import Subsystems.RoboSystem;
import Subsystems.Swerve;
import jaci.pathfinder.Trajectory;

public class FollowPathAction implements Action{
	private RoboSystem robot;
	
	private Trajectory trajectory;
	private boolean hasStarted;
	private double heading;
	private double desiredPortionToComplete = 0.98;
	
	public FollowPathAction(Trajectory trajectory){
		robot = RoboSystem.getInstance();
		this.trajectory = trajectory;
		hasStarted = false;
		heading = robot.pidgey.getAngle();
	}
	
	public FollowPathAction(Trajectory trajectory, double heading, double desiredPortionToComplete){
		this.trajectory = trajectory;
		hasStarted = false;
		robot = RoboSystem.getInstance();
		this.heading = heading;
		this.desiredPortionToComplete = desiredPortionToComplete;
	}
	
	@Override
	public boolean isFinished(){
		return hasStarted && robot.swerve.isFinishedWithPath(desiredPortionToComplete);
	}
	
	@Override
	public void update(){
		hasStarted = true;
	}
	
	@Override
	public void done(){
		robot.swerve.stop();
	}
	
	@Override
	public void start(){
		robot.swerve.followPath(trajectory, heading);
	}
	
}
