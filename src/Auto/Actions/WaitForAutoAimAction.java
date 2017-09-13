package Auto.Actions;

import Subsystems.RobotState;
import Subsystems.Turret;
import Vision.VisionServer;
import edu.wpi.first.wpilibj.Timer;

public class WaitForAutoAimAction implements Action{
	private Turret turret;
	private RobotState robotState;
	private VisionServer vision;
	private boolean isDone;
	private final double timeout = 3;
	private double startTime;
	public boolean timedOut;
	
	public WaitForAutoAimAction(){
		turret = Turret.getInstance();
		robotState = RobotState.getInstance();
		vision = VisionServer.getInstance();
		isDone = false;
		timedOut = false;
	}
	
	@Override
	public boolean isFinished(){
		return isDone;
	}
	
	@Override
	public void update(){
		if(turret.getCurrentState() == Turret.ControlState.VisionTracking && 
				turret.isStationary() && Math.abs(robotState.getVisionAngle()) <= 2.0 &&
				vision.isConnected()) isDone = true;
		
		if(Timer.getFPGATimestamp() - startTime > timeout){
			timedOut = true;
			isDone = true;
		}
	}
	
	@Override
	public void done(){
	}
	
	@Override
	public void start(){
		startTime = Timer.getFPGATimestamp();
		System.out.println("Waiting for auto aim");
	}
}
