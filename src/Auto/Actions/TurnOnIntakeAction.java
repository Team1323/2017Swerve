package Auto.Actions;

import Subsystems.RoboSystem;
import edu.wpi.first.wpilibj.Timer;

public class TurnOnIntakeAction implements Action{
	private boolean isDone;
	private final RoboSystem robot;
	
	public TurnOnIntakeAction(){
		isDone = false;
		robot = RoboSystem.getInstance();
	}
	
	@Override 
	public boolean isFinished(){
		return isDone;
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){	
	}
	
	@Override
	public void start(){
		robot.intake.intakeForward();
		isDone = true;
	}
}
