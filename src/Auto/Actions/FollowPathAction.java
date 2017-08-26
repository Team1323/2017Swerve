package Auto.Actions;

import Subsystems.Swerve;

public class FollowPathAction implements Action{
	private Swerve swerve;
	
	private Swerve.Path path;
	private boolean hasStarted;
	
	public FollowPathAction(Swerve.Path mPath){
		path = mPath;
		hasStarted = false;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return hasStarted && swerve.isFinishedWithPath();
	}
	
	@Override
	public void update(){
		hasStarted = true;
	}
	
	@Override
	public void done(){
		swerve.stop();
	}
	
	@Override
	public void start(){
		swerve.followPath(path);
	}
	
}
