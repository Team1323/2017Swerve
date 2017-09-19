package Auto.Actions;

import Subsystems.Swerve;

public class StraightPathAction implements Action{
	private Swerve swerve;
	
	private Swerve.Path path;
	private boolean hasStarted;
	private double wheelAngle;
	
	public StraightPathAction(Swerve.Path mPath, double wheelAngle){
		path = mPath;
		hasStarted = false;
		this.wheelAngle = wheelAngle;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return hasStarted && swerve.strictFinishedWithPath();
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
		swerve.setModuleAngles(wheelAngle);
		swerve.followPath(path, true);
	}
}
