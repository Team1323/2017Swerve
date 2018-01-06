package Loops;

import java.util.ArrayList;
import java.util.List;

import Subsystems.RobotState;
import Vision.TargetInfo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class LimelightProcessor implements Loop{
	static LimelightProcessor instance = new LimelightProcessor();
	RobotState robotState = RobotState.getInstance();
	
	public static LimelightProcessor getInstance(){
		return instance;
	}
	
	public LimelightProcessor(){
	}
	
	@Override 
	public void onStart(){
		
	}
	
	@Override 
	public void onLoop(){
		NetworkTable table = NetworkTable.getTable("limelight");
		double targetOffsetAngle_Horizontal = -table.getNumber("tx", 0);
		double targetOffsetAngle_Vertical = table.getNumber("ty", 0);
		double targetArea = table.getNumber("ta", 0);
		double targetSkew = table.getNumber("ts", 0);
		boolean targetInSight = (targetOffsetAngle_Horizontal != 0 || targetOffsetAngle_Vertical !=0 || 
				targetArea != 0 || targetSkew != 0) ? true : false;
		List<TargetInfo> targets = new ArrayList<TargetInfo>(1);
		if(targetInSight){
			targets.add(new TargetInfo(Math.tan(Math.toRadians(targetOffsetAngle_Horizontal)), Math.tan(Math.toRadians(targetOffsetAngle_Vertical))));
		}
		//robotState.addLimeLightUpdate(Timer.getFPGATimestamp(), targetOffsetAngle_Horizontal, targetOffsetAngle_Vertical, targetInSight);
		robotState.addVisionUpdate(Timer.getFPGATimestamp() - 0.006, targets);
	}
	
	@Override
	public void onStop(){
		
	}
}
