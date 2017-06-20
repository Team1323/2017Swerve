package Subsystems;

public class RoboSystem {
	static RoboSystem instance = new RoboSystem();
	public Swerve swerve = Swerve.getInstance();
	public Intake intake = Intake.getInstance();
	public static RoboSystem getInstance(){
		return instance;
	}
	public RoboSystem(){
		
	}
}
