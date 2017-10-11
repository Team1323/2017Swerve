package Subsystems;

import java.util.Optional;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.ShooterAimingParameters;
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
	private double stateStartTime = 0.0;
	private double lastTrackWhileShooting = Double.POSITIVE_INFINITY;
	
	public double goalX = 0.0;
	public double goalY = 0.0;
	
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
    	motor.changeControlMode(TalonControlMode.MotionMagic);
    	motor.set(0);
    	motor.setPID(4.0, 0.0, 160.0, 0.0, 0, 0.0, 0);
    	motor.setPID(4.0, 0.0, 100.0, 0.8184, 0, 0.0, 1);
		motor.setMotionMagicCruiseVelocity(300);
		motor.setMotionMagicAcceleration(800);
    	motor.setProfile(0);
		motor.enableBrakeMode(true);
		motor.setNominalClosedLoopVoltage(12);
		motor.enableForwardSoftLimit(true);
		motor.enableReverseSoftLimit(true);
		motor.setForwardSoftLimit((Constants.TURRET_MAX_ANGLE/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV);
		motor.setReverseSoftLimit((-Constants.TURRET_MAX_ANGLE/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV);
	}
	public enum ControlState{
		Off, VisionTracking, Manual, GyroComp, CalculatedTracking, MotionMagic, TrackingWhileShooting, Locked
	}
	public ControlState currentState = ControlState.MotionMagic;
	public static Turret getInstance(){
		return instance;
	}	
	public void setState(ControlState newState){
		currentState = newState;
		stateStartTime = Timer.getFPGATimestamp();
	}
	public ControlState getCurrentState(){
		return currentState;
	}
	
	public void setAngle(double angle){
		motor.changeControlMode(TalonControlMode.MotionMagic);
		if(angle > Constants.TURRET_MAX_ANGLE)
			angle = Constants.TURRET_MAX_ANGLE;
		if(angle < -Constants.TURRET_MAX_ANGLE)
			angle = -Constants.TURRET_MAX_ANGLE;
		motor.set(angle*Constants.TURRET_REVS_PER_DEGREE);
	}
	public void setMotionMagic(double angle){
		setState(ControlState.MotionMagic);
		motor.changeControlMode(TalonControlMode.MotionMagic);
		motor.setProfile(1);
		motor.set(angle*Constants.TURRET_REVS_PER_DEGREE);
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
	public void fieldPositionLock(){
		goalX = swerve.getX() - (Math.sin(Math.toRadians(Util.boundAngle0to360Degrees(getFieldRelativeAngle() - 180)))*(robotState.getTargetDistance() + Constants.kCameraYOffset + 7.5));
		goalY = swerve.getY() - (Math.cos(Math.toRadians(Util.boundAngle0to360Degrees(getFieldRelativeAngle() - 180)))*(robotState.getTargetDistance() + Constants.kCameraYOffset + 7.5));
		setState(ControlState.CalculatedTracking);
	}
	public void faceTarget(){
		double deltaX = swerve.getX() - goalX;
		double deltaY = swerve.getY() - goalY;
		double fieldRelativeAngle = 180 + Math.toDegrees(Math.atan(deltaX/deltaY));
		setFieldRelativeAngle(fieldRelativeAngle);
		System.out.println("Angle " + Double.toString(fieldRelativeAngle));
	}
	public double getGoal(){
		return ((motor.getSetpoint()/Constants.TURRET_ENC_REVS_PER_ACTUAL_REV)*360);
	}
	public void lock(){
		setState(ControlState.Locked);
	}
	public void gyroLock(){
		setState(ControlState.GyroComp);
		gyroLockedHeading = pidgey.getAngle();
		gyroLockedTurretAngle = getAngle();
	}
	public void gyroLock(double lockedHeading, double lockedTurretAngle){
		setState(ControlState.GyroComp);
		gyroLockedHeading = lockedHeading;
		gyroLockedTurretAngle = lockedTurretAngle;
	}
	public void setGyroLockAngle(double lockedHeading, double lockedTurretAngle){
		setState(ControlState.MotionMagic);
		setAngle(lockedTurretAngle - (pidgey.getAngle() - lockedHeading));
	}
	public void enableVision(){
		setState(ControlState.VisionTracking);
		Optional<ShooterAimingParameters> params = RobotState.getInstance().getAimingParameters(Timer.getFPGATimestamp());
		if(params.isPresent()){
			setAngle(-params.get().getTurretAngle().getDegrees());
		}
	}
	public void trackWhileShooting(){
		moveDegrees(robotState.getSmoothedVisionAngle());
		lastTrackWhileShooting = Timer.getFPGATimestamp();
		setState(ControlState.TrackingWhileShooting);
	}
	public double getTrueVisionDistance(){
		double modVisionAngle = Math.toRadians(robotState.getVisionAngle());
		double distance = robotState.getTargetDistance();
		double ogVisionAngle = -Math.toRadians(robotState.getOriginalVisionAngle());
		double radius = Math.hypot(Constants.kCameraXOffset, Constants.kCameraYOffset);
		double chord = Math.sqrt((2*radius*radius) - (2*radius*radius*Math.cos(modVisionAngle)));
		double radiusToCamPlane = Math.atan(Constants.kCameraYOffset/Math.abs(Constants.kCameraXOffset));
		double radiusToCamRay = radiusToCamPlane + (Math.PI/2);
		double innerIsoscelesAngle = (Math.PI - modVisionAngle)/2;
		double theta = radiusToCamRay - innerIsoscelesAngle - ogVisionAngle;
		double trueDistance = Math.sqrt((distance*distance) + (chord*chord) - (2*distance*chord*Math.cos(theta)));
		return trueDistance;
	}
	public void update(){
		switch(currentState){
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
				motor.setProfile(1);
				if(onTarget() && isStationary() && robotState.getTargetVisbility()){
					Optional<ShooterAimingParameters> params = RobotState.getInstance().getAimingParameters(Timer.getFPGATimestamp());
					if(params.isPresent()){
						setAngle(-params.get().getTurretAngle().getDegrees());
					}
				}
				SmartDashboard.putString("Turret Control State", "VisionTracking");
				break;
			case TrackingWhileShooting:
				double time = Timer.getFPGATimestamp();
				if(onTarget() && isStationary() && time - lastTrackWhileShooting > (Constants.kMaxAngleAge + 0.25)){
					moveDegrees(robotState.getSmoothedVisionAngle());
					lastTrackWhileShooting = time;
				}
				SmartDashboard.putString("Turret Control State", "TrackingWhileShooting");
				break;
			case MotionMagic:
				motor.setProfile(1);
				SmartDashboard.putString("Turret Control State", "MotionMagic");
				break;
			case Locked:
				setPercentVBus(0);
				if((Timer.getFPGATimestamp() - stateStartTime) > 0.25){
					setAngle(getAngle());
					setState(ControlState.MotionMagic);
				}
				break;
			case Manual:
				SmartDashboard.putString("Turret Control State", "Manual");
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
		return (Math.abs(getError()) < 1.0) && isStationary();
	}
	public double getTurretDegreesPerSecond(){
		return motor.getSpeed()/Constants.TURRET_REVS_PER_DEGREE/60;
	}
	public boolean isStationary(){
		return Math.abs(getTurretDegreesPerSecond()) < 1; 
	}
	public boolean isTracking(){
		return getCurrentState() == ControlState.VisionTracking;
	}
	public void resetAngle(double angle){
		motor.setPosition((angle/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV);
	}
	public synchronized void setPercentVBus(double speed){
		motor.changeControlMode(TalonControlMode.PercentVbus);
		motor.set(speed);
	}
	public double turretAngleToWheelAngle(double turretAngle){
		return turretAngle - 90;
	}
	private final Loop turretLoop = new Loop(){
		@Override
		public void onStart(){
			stop();
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
		SmartDashboard.putNumber("Turret Speed", getTurretDegreesPerSecond());
		SmartDashboard.putNumber("Turret Current", motor.getOutputCurrent());
		SmartDashboard.putNumber("Turret Voltage", motor.getOutputVoltage());
		Optional<ShooterAimingParameters> params = RobotState.getInstance().getAimingParameters(Timer.getFPGATimestamp());
		if(params.isPresent()){
			SmartDashboard.putNumber("True Vision Distance", params.get().getRange());
		}
		//SmartDashboard.putNumber("True Vision Distance", getTrueVisionDistance());
	}
}
