package Subsystems;

import com.ctre.PigeonImu;
import com.ctre.PigeonImu.PigeonState;

import Loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pidgeon {
	private PigeonImu pidgey;
	private Hanger hanger;
    private double currentAngle = 0.0;
    boolean pidgeonIsGood = false;
    double currentAngularRate = 0.0;
	public Pidgeon(){
		hanger = Hanger.getInstance();
		try{
			pidgey = new PigeonImu(hanger.getTalon());
			pidgey.EnableTemperatureCompensation(true);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	private static Pidgeon instance = null;
	public static Pidgeon getInstance(){
		if(instance == null){
			instance = new Pidgeon();
		}
		return instance;
	}
	public void update(){
		try{
			PigeonImu.GeneralStatus genStatus = new PigeonImu.GeneralStatus();
			PigeonImu.FusionStatus fusionStatus = new PigeonImu.FusionStatus();
			double [] xyz_dps = new double [3];
			currentAngle = -pidgey.GetFusedHeading(fusionStatus);
			pidgey.GetGeneralStatus(genStatus);
			pidgey.GetRawGyro(xyz_dps);
			pidgeonIsGood = (pidgey.GetState() == PigeonState.Ready) ? true : false;
			currentAngularRate = -xyz_dps[2];
			
			short [] ba_xyz = new short [3];
			pidgey.GetBiasedAccelerometer(ba_xyz);
			//SmartDashboard.putNumber("AccX", ba_xyz[0]);
			//SmartDashboard.putNumber("AccY", ba_xyz[1]);
			//SmartDashboard.putNumber("AccZ", ba_xyz[2]);
			
			double [] ypr = new double [3];
			pidgey.GetYawPitchRoll(ypr);
			//currentAngle = -ypr[0];
			
		}catch(Exception e){
			System.out.println(e);
		}
	}
	private final Loop pidgeonLoop = new Loop(){
		@Override
		public void onStart(){
			
		}
		@Override
		public void onLoop(){
			update();
		}
		@Override
		public void onStop(){
			
		}
	};
	public Loop getLoop(){
		return pidgeonLoop;
	}
	public boolean isGood(){
		return pidgeonIsGood;
	}
	public double getAngle(){
		return currentAngle;
	}
	public double getRealMathAngle(){
		return -currentAngle;
	}
	public double getAngularRate(){
		return currentAngularRate;
	}
	public void setAngle(int i){
		if(i == 0){
			pidgey.SetFusedHeading(i);
			pidgey.SetYaw(i);
		}else{
			pidgey.SetFusedHeading(360-i);
			pidgey.SetYaw(360-i);
		}
	}
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber(" Heading Angle ", getAngle());
		SmartDashboard.putNumber(" Pigeon Rate ", getAngularRate());
		SmartDashboard.putBoolean(" Pigeon Good ", isGood());
		SmartDashboard.putNumber("Pigeon Temp", pidgey.GetTemp());
	}
}
