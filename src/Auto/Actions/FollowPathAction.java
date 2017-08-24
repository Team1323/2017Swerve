package Auto.Actions;

import Subsystems.Swerve;

public class FollowPathAction implements Action{
	private Swerve swerve = Swerve.getInstance();
	
	private Swerve.Path path;
	private boolean hasStarted;
	
	public FollowPathAction(Swerve.Path mPath){
		path = mPath;
		hasStarted = false;
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
