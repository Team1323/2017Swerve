package Auto.Actions;

import Subsystems.RoboSystem;
import edu.wpi.first.wpilibj.Timer;

public class ReciprocateBallFlapAction implements Action{
	private double interval;
	private double timeForAction;
	private int reciprocations;
	private int reciprocationsCompleted;
	private RoboSystem robot;
	
	public ReciprocateBallFlapAction(double interval, int reciprocations){
		this.interval = interval;
		this.reciprocations = reciprocations * 2;
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return reciprocationsCompleted >= reciprocations;
	}
	
	@Override
	public void update(){
		if(Timer.getFPGATimestamp() > timeForAction){
			robot.toggleBallFlap();
			timeForAction += interval;
			reciprocationsCompleted++;
		}
	}
	
	@Override
	public void done(){
		
	}
	
	@Override
	public void start(){
		robot.toggleBallFlap();
		timeForAction = Timer.getFPGATimestamp() + interval;
		reciprocationsCompleted = 1;
	}
}
