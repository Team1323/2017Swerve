package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //added

public class Turret extends Subsystem{
	private static Turret instance = new Turret();
	private static Pidgeon pidgey;
	private static Swerve swerve;
	private CANTalon motor;
	private double gyroLockedHeading = 0.0;
	private double gyroLockedTurretAngle = 0.0;
	private int onTargetCheck = 0;
	
	public static final double goalX = -7.0;
	public static final double goalY = -1.0;
	
	private static RobotState robotState;
	
	public Turret(){
		pidgey = Pidgeon.getInstance();
		swerve = Swerve.getInstance();
		robotState = RobotState.getInstance();
		motor = new CANTalon(Ports.TURRET);
    	motor.setEncPosition(0);
    	motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	motor.reverseSensor(false);
    	motor.reverseOutput(false);
    	motor.configEncoderCodesPerRev(360);
    	motor.configNominalOutputVoltage(+0f, -0f);
    	motor.configPeakOutputVoltage(+5f, -5f);
    	motor.setAllowableClosedLoopErr(0); 
    	motor.changeControlMode(TalonControlMode.Position);
    	motor.set(0);
    	motor.setPID(4.0, 0.0, 320.0, 0.0, 0, 0.0, 0);	//practice bot pid tuning
    	motor.setPID(5.0, 0.00, Constants.TURRET_SMALL_D, 0.0, 0, 0.0, 1);
    	motor.setProfile(0);
		motor.enableBrakeMode(true);
		motor.setNominalClosedLoopVoltage(12);
		motor.enableForwardSoftLimit(true);
		motor.enableReverseSoftLimit(true);
		motor.setForwardSoftLimit((Constants.TURRET_MAX_ANGLE/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV);
		motor.setReverseSoftLimit((-Constants.TURRET_MAX_ANGLE/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV);
		if(motor.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent){
			DriverStation.reportError("Could not detect turret encoder!", false);
		}
	}
	public enum ControlState{
		Off, VisionTracking, Manual, GyroComp, AngleSnap, CalculatedTracking
	}
	public ControlState currentState = ControlState.AngleSnap;
	public static Turret getInstance(){
		return instance;
	}	
	public void setState(ControlState newState){
		currentState = newState;
	}
	public ControlState getCurrentState(){
		return currentState;
	}
	
	public void setAngle(double angle){
		motor.changeControlMode(TalonControlMode.Position);
		if(angle > Constants.TURRET_MAX_ANGLE)
			angle = Constants.TURRET_MAX_ANGLE;
		if(angle < -Constants.TURRET_MAX_ANGLE)
			angle = -Constants.TURRET_MAX_ANGLE;
		motor.set(angle*Constants.TURRET_REVS_PER_DEGREE);
		onTargetCheck = Constants.TURRET_ONTARGET_THRESH;
	}
	public void setSnapAngle(double targetAngle){
		setState(ControlState.AngleSnap);
		setAngle(targetAngle);
	}
	public void moveDegrees(double degree){
		double newAngle = getAngle() + degree;
		setAngle(newAngle);
	}
	public double getAngle(){
		return ((motor.getPosition()/Constants.TURRET_ENC_REVS_PER_ACTUAL_REV)*360);
	}
	public double getFieldRelativeAngle(){
		return (getAngle() - 90) + Util.boundAngle0to360Degrees(pidgey.getAngle());
	}
	public void setFieldRelativeAngle(double fieldAngle){
		setAngle(Util.boundAngleNeg180to180Degrees(fieldAngle + 90) - Util.boundAngleNeg180to180Degrees(pidgey.getAngle()));
	}
	public void faceTarget(){
		double deltaX = swerve.getX() - goalX;
		double deltaY = swerve.getY() - goalY;
		double fieldRelativeAngle = 180 + Math.toDegrees(Math.atan(deltaX/deltaY));
		setFieldRelativeAngle(fieldRelativeAngle);
	}
	public double getGoal(){
		return ((motor.getSetpoint()/Constants.TURRET_ENC_REVS_PER_ACTUAL_REV)*360);
	}
	public void lock(){
		setState(ControlState.AngleSnap);
		setAngle(getAngle());
	}
	public void gyroLock(){
		setState(ControlState.GyroComp);
		gyroLockedHeading = pidgey.getAngle();
		gyroLockedTurretAngle = getAngle();
	}
	public void enableVision(){
		setState(ControlState.VisionTracking);
		moveDegrees(robotState.getVisionAngle());
	}
	public void update(){
		switch(currentState){
			case AngleSnap:
				/*if(Math.abs(getError()) < Constants.TURRET_SMALL_PID_THRESH){
					motor.setProfile(1);
				}else{
					motor.setProfile(0);
				}*/
				motor.setProfile(0);
				SmartDashboard.putString("Turret Control State", "AngleSnap");
				break;
			case GyroComp:
				motor.setProfile(1);
				setAngle(gyroLockedTurretAngle - (pidgey.getAngle() - gyroLockedHeading));
				SmartDashboard.putString("Turret Control State", "GyroComp");
				break;
			case CalculatedTracking:
				motor.setProfile(1);
				faceTarget();
				SmartDashboard.putString("Turret Control State", "CalculatedTracking");
				break;
			case VisionTracking:
				motor.setProfile(0);
				if(onTarget()){
					//moveDegrees(robotState.getVisionAngle());
					//moveDegrees(-robotState.getAimingParameters(Timer.getFPGATimestamp()).getTurretAngle().getDegrees());
				}
				SmartDashboard.putString("Turret Control State", "VisionTracking");
				break;
			case Off:
				setPercentVBus(0);
				SmartDashboard.putString("Turret Control State", "Off");
				break;
			default:
				break;
		}
	}
	public double getError(){
		return (getGoal() - getAngle());
	}
	public boolean onTarget(){
		if(Math.abs(getError()) < 1.0){
			onTargetCheck--;
		}else{
			onTargetCheck = Constants.TURRET_ONTARGET_THRESH;
		}
		return onTargetCheck <= 0;
	}
	public void resetAngle(double angle){
		motor.setPosition((angle/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV);
	}
	public synchronized void setPercentVBus(double speed){
		motor.changeControlMode(TalonControlMode.PercentVbus);
		motor.set(speed);
	}
	private final Loop turretLoop = new Loop(){
		@Override
		public void onStart(){
			lock();
		}
		@Override
		public void onLoop(){
			update();
		}
		@Override
		public void onStop(){
			stop();
		}
	};
	public Loop getLoop(){
		return turretLoop;
	}
	@Override
	public synchronized void stop(){
		setState(ControlState.Off);
		setPercentVBus(0);
	}
	@Override 
	public synchronized void zeroSensors(){
		resetAngle(0);
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Turret Angle", getAngle());
		SmartDashboard.putNumber("Turret Field Relative Angle", getFieldRelativeAngle());
		SmartDashboard.putNumber("Turret Position", motor.getPosition());
		SmartDashboard.putNumber("Turret Encoder Position", motor.getEncPosition());
		SmartDashboard.putNumber("Turret Goal", getGoal());
		SmartDashboard.putNumber("Turret Error", getError());
		SmartDashboard.putNumber("Turret Current", motor.getOutputCurrent());
		SmartDashboard.putNumber("Turret Voltage", motor.getOutputVoltage());
	}
}
