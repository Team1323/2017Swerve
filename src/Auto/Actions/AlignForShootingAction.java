package Auto.Actions;

import java.util.Optional;

import Subsystems.RoboSystem;
import Subsystems.RobotState;
import Utilities.Constants;
import Utilities.ShooterAimingParameters;
import Utilities.Util;
import edu.wpi.first.wpilibj.Timer;

public class AlignForShootingAction implements Action{
	private RoboSystem robot;
	public boolean noVision = false;
	
	public AlignForShootingAction(){
		robot = RoboSystem.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return (robot.turret.onTarget() && robot.swerve.distanceOnTarget()) || noVision;
	}
	
	@Override
	public void start(){
		Optional<ShooterAimingParameters> params = RobotState.getInstance().getAimingParameters(Timer.getFPGATimestamp());
		if(params.isPresent()){
			robot.swerve.moveDistance(Util.boundAngle0to360Degrees(robot.turret.turretAngleToWheelAngle(-params.get().getTurretAngle().getDegrees())), params.get().getRange() - Constants.kOptimalShootingDistance);
			robot.turret.enableVision();
		}else{
			noVision = true;
		}
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
}
