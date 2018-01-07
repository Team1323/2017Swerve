package Subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.ShooterAimingParameters;
import Utilities.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //added

public class Turret extends Subsystem{
	private static Turret instance = new Turret();
	private static Pidgeon pidgey;
	private static Swerve swerve;
	private TalonSRX motor;
	private double gyroLockedHeading = 0.0;
	private double gyroLockedTurretAngle = 0.0;
	private double stateStartTime = 0.0;
	private double lastTrackWhileShooting = Double.POSITIVE_INFINITY;
	private double setpoint = 0.0;
	
	public double goalX = 0.0;
	public double goalY = 0.0;
	
	private static RobotState robotState;
	
	public Turret(){
		pidgey = Pidgeon.getInstance();
		swerve = Swerve.getInstance();
		robotState = RobotState.getInstance();
		motor = new TalonSRX(Ports.TURRET);
    	motor.getSensorCollection().setQuadraturePosition(0, 10);
    	motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	motor.setSensorPhase(false);
    	motor.setInverted(false);
    	//motor.configEncoderCodesPerRev(360);
    	motor.configNominalOutputForward(0.0, 10);
    	motor.configNominalOutputReverse(0.0, 10);
    	motor.configPeakOutputForward(5.0/12.0, 10);
    	motor.configPeakOutputReverse(-5.0/12.0, 10);
    	motor.configAllowableClosedloopError(0, 0, 10);
    	motor.set(ControlMode.PercentOutput, 0);
    	/*motor.setPID(4.0, 0.0, 160.0, 0.0, 0, 0.0, 0);
    	motor.setPID(4.0, 0.0, 100.0, 0.8184, 0, 0.0, 1);*/
    	motor.selectProfileSlot(0, 0);
    	motor.config_kP(0, 4.0, 10);
    	motor.config_kI(0, 0.0, 10);
    	motor.config_kD(0, 100.0, 10);
    	motor.config_kF(0, 0.8184, 10);
		motor.configMotionCruiseVelocity(300, 10);
		motor.configMotionAcceleration(800, 10);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.enableVoltageCompensation(true);
		motor.configForwardSoftLimitEnable(true, 10);
		motor.configReverseSoftLimitEnable(true, 10);
		motor.configForwardSoftLimitThreshold((int) ((Constants.TURRET_MAX_ANGLE/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV), 10);
		motor.configReverseSoftLimitThreshold((int) ((-Constants.TURRET_MAX_ANGLE/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV), 10);
	}
	public enum ControlState{
		Off, VisionTracking, Manual, GyroComp, CalculatedTracking, MotionMagic, TrackingWhileShooting, Locked
	}
	public ControlState currentState = ControlState.Off;
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
		if(angle > Constants.TURRET_MAX_ANGLE)
			angle = Constants.TURRET_MAX_ANGLE;
		if(angle < -Constants.TURRET_MAX_ANGLE)
			angle = -Constants.TURRET_MAX_ANGLE;
		motor.set(ControlMode.MotionMagic, angle*Constants.TURRET_REVS_PER_DEGREE);
		setpoint = angle*Constants.TURRET_REVS_PER_DEGREE;
	}
	public void setMotionMagic(double angle){
		setState(ControlState.MotionMagic);
		motor.set(ControlMode.MotionMagic, angle*Constants.TURRET_REVS_PER_DEGREE);
		setpoint = angle*Constants.TURRET_REVS_PER_DEGREE;
	}
	public void moveDegrees(double degree){
		double newAngle = getAngle() + degree;
		setAngle(newAngle);
	}
	public double getAngle(){
		return ((motor.getSelectedSensorPosition(0)/Constants.TURRET_ENC_REVS_PER_ACTUAL_REV)*360);
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
		return ((setpoint/Constants.TURRET_ENC_REVS_PER_ACTUAL_REV)*360);
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
				setAngle(gyroLockedTurretAngle - (pidgey.getAngle() - gyroLockedHeading));
				SmartDashboard.putString("Turret Control State", "GyroComp");
				break;
			case CalculatedTracking:
				faceTarget();
				SmartDashboard.putString("Turret Control State", "CalculatedTracking");
				break;
			case VisionTracking:
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
		return motor.getSelectedSensorVelocity(0)/Constants.TURRET_REVS_PER_DEGREE/60;
	}
	public boolean isStationary(){
		return Math.abs(getTurretDegreesPerSecond()) < 1; 
	}
	public boolean isTracking(){
		return getCurrentState() == ControlState.VisionTracking;
	}
	public void resetAngle(double angle){
		motor.setSelectedSensorPosition((int) ((angle/360)*Constants.TURRET_ENC_REVS_PER_ACTUAL_REV), 0, 10);
	}
	public synchronized void setPercentVBus(double speed){
		motor.set(ControlMode.PercentOutput, speed);
		setpoint = speed;
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
		SmartDashboard.putNumber("Turret Position", motor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Turret Encoder Position", motor.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Turret Goal", getGoal());
		SmartDashboard.putNumber("Turret Error", getError());
		SmartDashboard.putNumber("Turret Speed", getTurretDegreesPerSecond());
		SmartDashboard.putNumber("Turret Current", motor.getOutputCurrent());
		SmartDashboard.putNumber("Turret Voltage", motor.getMotorOutputVoltage());
		Optional<ShooterAimingParameters> params = RobotState.getInstance().getAimingParameters(Timer.getFPGATimestamp());
		if(params.isPresent()){
			SmartDashboard.putNumber("True Vision Distance", params.get().getRange());
		}
		//SmartDashboard.putNumber("True Vision Distance", getTrueVisionDistance());
	}
}
