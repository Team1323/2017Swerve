package Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Utilities.Constants;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.Translation2d;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveModule extends Subsystem{
	static final double HALF_LENGTH = Constants.WHEELBASE_LENGTH/2;
	static final double HALF_WIDTH = Constants.WHEELBASE_WIDTH/2;
	
	private Pidgeon pidgey;
	
	private TalonSRX rotationMotor;
	public TalonSRX driveMotor;
	private int moduleID;
	public int arrayIndex(){
		return moduleID - 1;
	}
	private int absolutePosition;
	private double offset = 0.0;
	public double pathFollowingOffset = 0.0;
	private double currentDistance = 0.0;
	private double lastDistance = 0.0;
	private double currentX = 0;
	private double currentY = 0;
	private double defaultX = 0;
	private double defaultY = 0;
	public boolean hasBraked = false;
	public boolean hasBraked(){return hasBraked;}
	private boolean isReversed = false;
	public void reverseOpenLoop(boolean reversed){
		isReversed = reversed;
	}
	private double rotationSetpoint = 0.0;
	private double driveSetpoint = 0.0;
	public SwerveDriveModule(int rotationMotorPort, int driveMotorPort,int moduleNum,double _offSet){
		rotationMotor = new TalonSRX(rotationMotorPort);
		driveMotor = new TalonSRX(driveMotorPort);
		moduleID = moduleNum;  
		offset = _offSet;
		loadProperties();
		pidgey = Pidgeon.getInstance();
	}
	public final void loadProperties(){
    	//absolutePosition = rotationMotor.getSensorCollection().getPulseWidthPosition() & 0xFFF;
		absolutePosition = rotationMotor.getSensorCollection().getAnalogInRaw();
    	//rotationMotor.getSensorCollection().setAnalogPosition(absolutePosition, 10);
    	rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
    	rotationMotor.setSelectedSensorPosition(absolutePosition, 0, 10);
    	rotationMotor.setSensorPhase(true);
    	rotationMotor.setInverted(true);
    	//rotationMotor.configPotentiometerTurns(360);
    	rotationMotor.configNominalOutputForward(0.0, 10);
    	rotationMotor.configNominalOutputReverse(0.0, 10);
    	rotationMotor.configPeakOutputForward(7.0/12.0, 10);
    	rotationMotor.configPeakOutputReverse(-7.0/12.0, 10);
    	rotationMotor.configAllowableClosedloopError(0, 0, 10);
    	rotationMotor.configMotionAcceleration(63070*10, 10);
    	rotationMotor.configMotionCruiseVelocity(63070, 10);
    	/*rotationMotor.setPID(5.0, 0.0, 40.0, 0.0, 0, 0.0, 0);
    	rotationMotor.setPID(5.0, 0.0, 160.0, 1.705, 0, 0.0, 1);
    	rotationMotor.setProfile(1);*/
    	rotationMotor.selectProfileSlot(0, 0);
    	rotationMotor.config_kP(0, 5.0, 10);
    	rotationMotor.config_kI(0, 0.0, 10);
    	rotationMotor.config_kD(0, 160.0, 10);
    	rotationMotor.config_kF(0, 1.705, 10);
    	rotationMotor.set(ControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));
    	driveMotor.getSensorCollection().setQuadraturePosition(0, 10);
    	driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    	driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
    	driveMotor.configNominalOutputForward(0.0, 10);
    	driveMotor.configNominalOutputReverse(0.0, 10);
    	driveMotor.enableVoltageCompensation(true);
    	driveMotor.configOpenloopRamp(0, 10);
    	driveMotor.configAllowableClosedloopError(0, 0, 10);
    	driveMotor.setInverted(false);
    	driveMotor.setNeutralMode(NeutralMode.Brake);
    	driveMotor.selectProfileSlot(1, 0);
    	driveMotor.config_kP(0, 1.2, 10);
    	driveMotor.config_kI(0, 0.0, 10);
    	driveMotor.config_kD(0, 21.0, 10);
    	driveMotor.config_kF(0, 0.10685189, 10);
    	driveMotor.selectProfileSlot(0, 0);
    	driveMotor.config_kP(0, 0.5, 10);
    	driveMotor.config_kI(0, 0.0, 10);
    	driveMotor.config_kD(0, 80.0, 10);
    	driveMotor.config_kF(0, 0.10685189, 10);
    	driveMotor.configMotionCruiseVelocity(3200, 10);
    	driveMotor.configMotionAcceleration(4500, 10);
	}
	public double degreesToEncUnits(double degrees){
		return degrees/360.0*1024.0;
	}
	public double encUnitsToDegrees(double encUnits){
		return encUnits/1024.0*360.0;
	}
	public double getRawAngle(){
		return encUnitsToDegrees(rotationMotor.getSelectedSensorPosition(0));
	}
	public double getModuleAngle(){
		return Util.boundAngle0to360Degrees(getRawAngle() - offset);
	}
	public double getFieldRelativeAngle(){
		return Util.boundAngle0to360Degrees(-getModuleAngle() - Util.boundAngle0to360Degrees(pidgey.getAngle()));
	}
	public void setFieldRelativeAngle(double fieldAngle){
		setModuleAngle(Util.boundAngle0to360Degrees(fieldAngle - Util.boundAngle0to360Degrees(pidgey.getAngle())));
	}
	public void setModuleAngle(double goalAngle){
		double newAngle = Util.placeInAppropriate0To360Scope(getRawAngle(), goalAngle+offset);
		rotationMotor.set(ControlMode.MotionMagic, degreesToEncUnits(newAngle));
		rotationSetpoint = degreesToEncUnits(newAngle);
	}
	public void setRotationOpenLoop(double power){
		rotationMotor.set(ControlMode.PercentOutput, power);
		rotationSetpoint = power;
	}
	public double getGoal(){
		return Util.boundAngle0to360Degrees(rotationSetpoint);
	}
	public void setDriveOpenLoop(double power){
		if(isReversed){
			driveMotor.set(ControlMode.PercentOutput, -power);
			driveSetpoint = -power;
		}else{
			driveMotor.set(ControlMode.PercentOutput, power);
			driveSetpoint = power;
		}
	}
	public void setDriveVoltage(double voltage){
		setDriveOpenLoop(voltage/12.0);
	}
	public void moveInches(double inches){
		driveMotor.selectProfileSlot(0, 0);
    	driveMotor.set(ControlMode.MotionMagic, driveMotor.getSelectedSensorPosition(0) + inchesToEncUnits(inches));
    	driveSetpoint = driveMotor.getSelectedSensorPosition(0) + inchesToEncUnits(inches);
	}
	public void lockPosition(){
		driveMotor.selectProfileSlot(0, 0);
		driveMotor.set(ControlMode.MotionMagic, driveMotor.getSelectedSensorPosition(0));
		driveSetpoint = driveMotor.getSelectedSensorPosition(0);
	}
	public void setDriveVelocity(double inchesPerSecond){
		driveMotor.selectProfileSlot(1, 0);
		//driveMotor.set(ControlMode.Velocity, inchesPerSecondToRpm(inchesPerSecond));
		//driveSetpoint = inchesPerSecondToRpm(inchesPerSecond);
	}
	public double getEncoderDistanceInches(){
		double inches = driveMotor.getSelectedSensorPosition(0)/Constants.SWERVE_ENC_UNITS_PER_INCH;
		return (moduleID == 1) ? -inches : inches;
	}
	public double getEncoderDistanceFeet(){
		return getEncoderDistanceInches()/12;
	}
	public double getModuleInchesPerSecond(){
		return driveMotor.getSelectedSensorVelocity(0)/Constants.SWERVE_ENC_UNITS_PER_INCH/60;
	}
	public double getModuleFeetPerSecond(){
		return getModuleInchesPerSecond()/12;
	}
	public double encUnitsToInches(int encUnits){
		return encUnits/Constants.SWERVE_ENC_UNITS_PER_INCH;
	}
	public int inchesToEncUnits(double inches){
		return (int) (inches*Constants.SWERVE_ENC_UNITS_PER_INCH);
	}
	public boolean onDistanceTarget(){
		return (driveMotor.getControlMode() == ControlMode.MotionMagic) && 
				Math.abs(encUnitsToInches((int)driveSetpoint) - encUnitsToInches(driveMotor.getSelectedSensorPosition(0))) < 2.0;
	}
	public void setOriginCoordinates(double x, double y){
		currentX = x;
		currentY = y;
		defaultX = x;
		defaultY = y;
	}
	public double getX(){return currentX;}
	public double getY(){return currentY;}
	public void update(){
		currentDistance = getEncoderDistanceInches();
		double distanceTraveled = currentDistance - lastDistance;
		double angle = Math.toRadians(getFieldRelativeAngle());
		currentX += Math.cos(angle) * distanceTraveled;
		currentY += Math.sin(angle) * distanceTraveled;
		lastDistance = currentDistance;
	}
	@Override
	public synchronized void stop(){
		setRotationOpenLoop(0.0);
		setDriveOpenLoop(0.0);
	}
	@Override
	public synchronized void zeroSensors(){
		zeroSensors(90);
	}
	public synchronized void zeroSensors(double heading){
		driveMotor.setSelectedSensorPosition(0, 0, 10);
		RigidTransform2d robotPose = RigidTransform2d.fromRotation(Rotation2d.fromDegrees(heading));
		RigidTransform2d modulePose = robotPose.transformBy(RigidTransform2d.fromTranslation(new Translation2d(defaultX, defaultY)));
		currentX = modulePose.getTranslation().x();
		currentY = modulePose.getTranslation().y();
		currentDistance = 0;
		lastDistance = 0;
	}
	@Override
	public void outputToSmartDashboard(){
		String smallX = Float.toString((float)(Math.round(getX() * 100.0) / 100.0));
		String smallY = Float.toString((float)(Math.round(getY() * 100.0) / 100.0));
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Angle", getModuleAngle()/*rotationMotor.getSelectedSensorPosition(0)*/);
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Goal", rotationSetpoint);
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Position", getEncoderDistanceInches());
		SmartDashboard.putString("Module " + Integer.toString(moduleID) + " Coordinates ", "("+smallX+" , "+smallY+")");
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + "Speed", getModuleFeetPerSecond());
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Error", driveSetpoint - driveMotor.getSelectedSensorVelocity(0));
		if(moduleID == 4){
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " X", currentX);
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Y", currentY);
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Field Relative Angle", getFieldRelativeAngle());
			
		}
	}
}
