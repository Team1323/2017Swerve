package Subsystems;

import Utilities.Ports;
import edu.wpi.first.wpilibj.Solenoid;

public class RoboSystem {
	static RoboSystem instance = null;
	
	public Intake intake;
	public Pidgeon pidgey;
	public Swerve swerve;
	public Turret turret;
	public Hanger hanger;
	public GearIntake gearIntake;
	public Shooter shooter;
	public Sweeper sweeper;
	
	public Solenoid ballFlap;
	
	public static RoboSystem getInstance(){
		if(instance == null){
			instance = new RoboSystem();
		}
		return instance;
	}
	public RoboSystem(){
		hanger = Hanger.getInstance();
		intake = Intake.getInstance();
		pidgey = Pidgeon.getInstance();
		swerve = Swerve.getInstance();
		turret = Turret.getInstance();
		gearIntake = GearIntake.getInstance();
		shooter = Shooter.getInstance();
		sweeper = Sweeper.getInstance();		
		ballFlap = new Solenoid(20, Ports.BALL_FLAP);
	}
	public void extendBallFlap(){
		ballFlap.set(true);
	}
	public void retractBallFlap(){
		ballFlap.set(false);
	}
	public void toggleBallFlap(){
		ballFlap.set(!ballFlap.get());
	}
}
