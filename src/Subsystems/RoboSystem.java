package Subsystems;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
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

	public CameraServer cam;
	
	public Solenoid ballFlap;
	
	public DigitalInput banner;
	
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
		ballFlap = new Solenoid(20, 2/*Ports.BALL_FLAP*/);
		banner = new DigitalInput(1);
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
	public boolean isBallFlapExtended(){
		return ballFlap.get();
	}
	public void initCamera(){
		cam = CameraServer.getInstance();
    	UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    	usbCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
    	MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    	mjpegServer2.setSource(usbCamera);
	}
	public boolean getBanner(){
		return banner.get();
	}
}
