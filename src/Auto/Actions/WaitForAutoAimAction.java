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
	private final double timeout = 1.5;
	private double startTime;
	private double startingAngle;
	private double pigeonAngle;
	private boolean timedOut;
	private int cyclesOnTarget = 0;
	private int cyclesForCompletion = 6;
	public boolean timedOut(){
		return timedOut;
	}
	
	public WaitForAutoAimAction(double startingAngle, double pigeonAngle){
		turret = Turret.getInstance();
		robotState = RobotState.getInstance();
		vision = VisionServer.getInstance();
		isDone = false;
		timedOut = false;
		this.startingAngle = startingAngle;
		this.pigeonAngle = pigeonAngle;
	}
	
	@Override
	public boolean isFinished(){
		return isDone;
	}
	
	@Override
	public void update(){
		if(turret.getCurrentState() == Turret.ControlState.VisionTracking && 
				turret.isStationary() && Math.abs(robotState.getVisionAngle()) <= 1.5 &&
				vision.isConnected()){ 
			cyclesOnTarget++;
			if(cyclesOnTarget >= cyclesForCompletion){
				isDone = true; 
				System.out.println("Vision Satisfied");
			}
		}else{
			cyclesOnTarget = 0;
		}
		
		if(Timer.getFPGATimestamp() - startTime > timeout){
			timedOut = true;
			isDone = true;
		}
	}
	
	@Override
	public void done(){
		if(timedOut){
			if(startingAngle == 90){
				System.out.println("Starting GyroLock");
				turret.setGyroLockAngle(-pigeonAngle, 96);
			}else if(startingAngle == -90){
				System.out.println("Starting GyroLock");
				turret.setGyroLockAngle(-pigeonAngle, -100);
			}
		}
	}
	
	@Override
	public void start(){
		startTime = Timer.getFPGATimestamp();
		System.out.println("Waiting for auto aim");
	}
}
