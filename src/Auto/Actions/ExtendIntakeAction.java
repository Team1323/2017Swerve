package Auto.Actions;

import Subsystems.RoboSystem;
import edu.wpi.first.wpilibj.Timer;

public class ExtendIntakeAction implements Action{
	private double startTime;
	private RoboSystem robot;
	
	public ExtendIntakeAction(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		if(Timer.getFPGATimestamp() - startTime >= 0.5){
			robot.intake.stop();
		}
		return Timer.getFPGATimestamp() - startTime >= 0.5;
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){
		robot.intake.stop();
	}
	
	@Override
	public void start(){
		startTime = Timer.getFPGATimestamp();
		robot.intake.intakeReverse();
	}
}
