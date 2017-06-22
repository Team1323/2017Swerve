package Subsystems;

import com.ctre.CANTalon;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.PigeonState;

public class Pidgeon {
	private PigeonImu pidgey;
    private double currentAngle = 0.0;
    boolean pidgeonIsGood = false;
    double currentAngularRate = 0.0;
	public Pidgeon(CANTalon talon){
		try{
			pidgey = new PigeonImu(talon);
		}catch(Exception e){
			System.out.println(e);
		}
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
			
		}catch(Exception e){
			System.out.println(e);
		}
	}
	public boolean isGood(){
		return pidgeonIsGood;
	}
	public double getAngle(){
		return currentAngle;
	}
	public double getAngularRate(){
		return currentAngularRate;
	}
	public void setAngle(double i){
		if(i != 0.0){
			pidgey.SetFusedHeading(360.0-i);
		}else{
			pidgey.SetFusedHeading(i);
		}
	}
}
