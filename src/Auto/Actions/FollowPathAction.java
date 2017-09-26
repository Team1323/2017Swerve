package Auto.Actions;

import Subsystems.RoboSystem;
import Subsystems.Swerve;

public class FollowPathAction implements Action{
	private RoboSystem robot;
	
	private Swerve.Path path;
	private boolean hasStarted;
	private double heading;
	
	public FollowPathAction(Swerve.Path mPath){
		robot = RoboSystem.getInstance();
		path = mPath;
		hasStarted = false;
		heading = robot.pidgey.getAngle();
	}
	
	public FollowPathAction(Swerve.Path mPath, double heading){
		path = mPath;
		hasStarted = false;
		robot = RoboSystem.getInstance();
		this.heading = heading;
	}
	
	@Override
	public boolean isFinished(){
		return hasStarted && robot.swerve.isFinishedWithPath();
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
		robot.swerve.followPath(path, heading);
	}
	
}
