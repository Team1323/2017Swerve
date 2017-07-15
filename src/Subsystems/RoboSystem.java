package Subsystems;

public class RoboSystem {
	static RoboSystem instance = new RoboSystem();
	public Swerve swerve = Swerve.getInstance();
	public Intake intake = Intake.getInstance();
	public Turret turret = Turret.getInstance();
	public Hanger hanger = Hanger.getInstance();
	public GearIntake gearIntake = GearIntake.getInstance();
	public Shooter shooter = Shooter.getInstance();
	public Sweeper sweeper = Sweeper.getInstance();
	public Pidgeon pidgey = Pidgeon.getInstance();
	public static RoboSystem getInstance(){
		return instance;
	}
	public RoboSystem(){
		
	}
}
