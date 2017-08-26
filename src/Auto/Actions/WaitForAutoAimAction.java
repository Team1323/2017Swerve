package Auto.Actions;

import Subsystems.RobotState;
import Subsystems.Turret;
import Vision.VisionServer;

public class WaitForAutoAimAction implements Action{
	private Turret turret;
	private RobotState robotState;
	private VisionServer vision;
	
	public WaitForAutoAimAction(){
		turret = Turret.getInstance();
		robotState = RobotState.getInstance();
		vision = VisionServer.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return (turret.getCurrentState() == Turret.ControlState.VisionTracking && 
				turret.isStationary() && Math.abs(robotState.getVisionAngle()) <= 2.0 &&
				vision.isConnected());
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){
	}
	
	@Override
	public void start(){
		System.out.println("Waiting for auto aim");
	}
}
