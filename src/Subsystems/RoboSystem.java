package Subsystems;

import Utilities.Ports;
import edu.wpi.first.wpilibj.Solenoid;

public class RoboSystem {
	static RoboSystem instance = null;
	public Intake intake = Intake.getInstance();
	public Pidgeon pidgey = Pidgeon.getInstance();
	public Swerve swerve = Swerve.getInstance();
	public Turret turret = Turret.getInstance();
	public Hanger hanger = Hanger.getInstance();
	public GearIntake gearIntake = GearIntake.getInstance();
	public Shooter shooter = Shooter.getInstance();
	public Sweeper sweeper = Sweeper.getInstance();
	
	public Solenoid ballFlap;
	
	public static RoboSystem getInstance(){
		if(instance == null){
			instance = new RoboSystem();
		}
		return instance;
	}
	public RoboSystem(){
		ballFlap = new Solenoid(Ports.BALL_FLAP);
	}
	public void extendBallFlap(){
		ballFlap.set(true);
	}
	public void retractBallFlap(){
		ballFlap.set(false);
	}
}
