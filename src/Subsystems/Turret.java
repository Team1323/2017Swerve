package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //added

public class Turret extends Subsystem{
	private static Turret instance = new Turret();
	private CANTalon motor;
	private double lockedAngle = 0.0;
	private double lockedTurretAngle = 0.0;
	private int onTargetCheck = 0;
	private double angleOffset = 0;
	public Turret(){
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
    	motor.setPID(Constants.TURRET_DEFAULT_P, 0.0, Constants.TURRET_DEFAULT_D, 0.0, 0, 0.0, 0);	//practice bot pid tuning
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
		Off, VisionTracking, Manual, GyroComp, AngleSnap
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
		setState(ControlState.AngleSnap);
		motor.changeControlMode(TalonControlMode.Position);
		if(angle > Constants.TURRET_MAX_ANGLE)
			angle = Constants.TURRET_MAX_ANGLE;
		if(angle < -Constants.TURRET_MAX_ANGLE)
			angle = -Constants.TURRET_MAX_ANGLE;
		motor.set(angle*Constants.TURRET_REVS_PER_DEGREE);
		onTargetCheck = Constants.TURRET_ONTARGET_THRESH;
	}
	public void moveDegrees(double degree){
		double newAngle = getAngle() - degree;
		setAngle(newAngle);
	}
	public double getAngle(){
		return ((motor.getPosition()/Constants.TURRET_ENC_REVS_PER_ACTUAL_REV)*360);
	}
	public double getGoal(){
		return ((motor.getSetpoint()/Constants.TURRET_ENC_REVS_PER_ACTUAL_REV)*360);
	}
	public void lock(){
		setAngle(getAngle());
	}
	public void update(){
		switch(currentState){
			case AngleSnap:
				if(Math.abs(getError()) < Constants.TURRET_SMALL_PID_THRESH){
					motor.setProfile(1);
				}else{
					motor.setProfile(0);
				}
				break;
			default:
				
				break;
		}
	}
	public double getError(){
		return (getGoal() - getAngle());
	}
	public boolean onTarget(){
		if(Math.abs(getError()) < 2.5){
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
		setPercentVBus(0);
	}
	@Override 
	public synchronized void zeroSensors(){
		resetAngle(0);
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Turret Angle", getAngle());
		SmartDashboard.putNumber("Turret Position", motor.getPosition());
		SmartDashboard.putNumber("Turret Encoder Position", motor.getEncPosition());
		SmartDashboard.putNumber("Turret Goal", getGoal());
		SmartDashboard.putNumber("Turret Error", getError());
		SmartDashboard.putNumber("Turret Current", motor.getOutputCurrent());
		SmartDashboard.putNumber("Turret Voltage", motor.getOutputVoltage());
	}
}