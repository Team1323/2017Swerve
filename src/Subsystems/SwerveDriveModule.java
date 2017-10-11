package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

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
	
	private CANTalon rotationMotor;
	public CANTalon driveMotor;
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
	public void reverseOutput(boolean reversed){
		isReversed = reversed;
	}
	public SwerveDriveModule(int rotationMotorPort, int driveMotorPort,int moduleNum,double _offSet){
		rotationMotor = new CANTalon(rotationMotorPort);
		driveMotor = new CANTalon(driveMotorPort);
		moduleID = moduleNum;  
		offset = _offSet;
		loadProperties();
		pidgey = Pidgeon.getInstance();
	}
	public final void loadProperties(){
    	absolutePosition = rotationMotor.getPulseWidthPosition() & 0xFFF;
    	rotationMotor.setEncPosition(absolutePosition);
    	rotationMotor.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
    	rotationMotor.reverseSensor(false);
    	rotationMotor.reverseOutput(true);
    	rotationMotor.configPotentiometerTurns(360);
    	rotationMotor.configNominalOutputVoltage(+0f, -0f);
    	rotationMotor.configPeakOutputVoltage(+7f, -7f);
    	rotationMotor.setAllowableClosedLoopErr(0); 
    	rotationMotor.changeControlMode(TalonControlMode.MotionMagic);
    	rotationMotor.setMotionMagicCruiseVelocity(63070);
    	rotationMotor.setMotionMagicAcceleration(63070*10);
    	rotationMotor.setPID(5.0, 0.0, 40.0, 0.0, 0, 0.0, 0);
    	rotationMotor.setPID(5.0, 0.0, 160.0, 1.705, 0, 0.0, 1);
    	rotationMotor.setProfile(1);
    	rotationMotor.set(rotationMotor.getPosition());
    	driveMotor.setEncPosition(0);
    	driveMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	driveMotor.configEncoderCodesPerRev(360);
    	driveMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
    	driveMotor.configNominalOutputVoltage(+0f, -0f);
    	driveMotor.setVoltageRampRate(0.0);
    	driveMotor.setAllowableClosedLoopErr(0);
    	driveMotor.changeControlMode(TalonControlMode.PercentVbus);
    	driveMotor.reverseOutput(false);
    	driveMotor.enableBrakeMode(true);
    	driveMotor.setProfile(0);
    	driveMotor.setPID(0.5, 0, 80.0, 0.10685189, 0, 0.0, 0);
    	driveMotor.setMotionMagicCruiseVelocity(3200);
    	driveMotor.setMotionMagicAcceleration(4500);
	}
	public double getRawAngle(){
		return rotationMotor.getPosition();
	}
	public double getModuleAngle(){
		return Util.boundAngle0to360Degrees(getRawAngle() - offset);
	}
	public double getFieldRelativeAngle(){
		return Util.boundAngle0to360Degrees(getModuleAngle() + Util.boundAngle0to360Degrees(pidgey.getAngle()));
	}
	public void setFieldRelativeAngle(double fieldAngle){
		setModuleAngle(Util.boundAngle0to360Degrees(fieldAngle - Util.boundAngle0to360Degrees(pidgey.getAngle())));
	}
	public void setModuleAngle(double goalAngle){
		rotationMotor.changeControlMode(TalonControlMode.MotionMagic);
		double newAngle = Util.placeInAppropriate0To360Scope(getRawAngle(), goalAngle+offset);
		rotationMotor.set(newAngle);
	}
	public void setRotationOpenLoop(double power){
		rotationMotor.changeControlMode(TalonControlMode.PercentVbus);
		rotationMotor.set(power);
	}
	public double getGoal(){
		return Util.boundAngle0to360Degrees(rotationMotor.getSetpoint() - offset);
	}
	public void setDriveSpeed(double power){
		driveMotor.changeControlMode(TalonControlMode.PercentVbus);
		if(isReversed){
			driveMotor.set(-power);
		}else{
			driveMotor.set(power);
		}
	}
	public void setVoltage(double voltage){
		driveMotor.changeControlMode(TalonControlMode.Voltage);
		if(isReversed){
			driveMotor.set(-voltage);
		}else{
			driveMotor.set(voltage);
		}
	}
	public double getEncoderDistanceInches(){
		return driveMotor.getPosition()/Constants.SWERVE_ENCODER_REVS_PER_INCH;
	}
	public double getEncoderDistanceFeet(){
		return (moduleID == 1) ? -getEncoderDistanceInches()/12 : getEncoderDistanceInches()/12;
	}
	public double getModuleInchesPerSecond(){
		return driveMotor.getSpeed()/Constants.SWERVE_ENCODER_REVS_PER_INCH/60;
	}
	public double getModuleFeetPerSecond(){
		return getModuleInchesPerSecond()/12;
	}
	public double inchesToRotations(double inches){
		return inches*Constants.SWERVE_ENCODER_REVS_PER_INCH;
	}
	public double rotationsToInches(double rotations){
		return rotations/Constants.SWERVE_ENCODER_REVS_PER_INCH;
	}
	public void moveInches(double inches){
		driveMotor.changeControlMode(TalonControlMode.MotionMagic);
    	driveMotor.set(driveMotor.getPosition() + inchesToRotations(inches));
	}
	public void lockPosition(){
		driveMotor.changeControlMode(TalonControlMode.MotionMagic);
		driveMotor.set(driveMotor.getPosition());
	}
	public boolean onDistanceTarget(){
		return (driveMotor.getControlMode() == TalonControlMode.MotionMagic) && 
				Math.abs(rotationsToInches(driveMotor.getSetpoint()) - rotationsToInches(driveMotor.getPosition())) < 1.0;
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
		double angle = Math.toRadians(90 - getFieldRelativeAngle());
		currentX += Math.cos(angle) * distanceTraveled;
		currentY += Math.sin(angle) * distanceTraveled;
		lastDistance = currentDistance;
		
		double halfDiagonal = Math.toDegrees(Math.atan(HALF_WIDTH/HALF_LENGTH));
		double robotAngle = Math.toRadians(180 - ((180-Util.boundAngle0to360Degrees(pidgey.getAngle()))+halfDiagonal));
		/*robotX = currentX + (Math.sin(robotAngle) * Math.hypot(HALF_LENGTH, HALF_WIDTH));
		robotY = currentY + (Math.cos(robotAngle) * Math.hypot(HALF_LENGTH, HALF_WIDTH));*/
		
	}
	@Override
	public synchronized void stop(){
		setRotationOpenLoop(0.0);
		setDriveSpeed(0.0);
	}
	@Override
	public synchronized void zeroSensors(){
		zeroSensors(90);
	}
	public synchronized void zeroSensors(double heading){
		driveMotor.setEncPosition(0);
		driveMotor.setPosition(0);
		RigidTransform2d robotPose = RigidTransform2d.fromRotation(Rotation2d.fromDegrees(heading));
		RigidTransform2d modulePose = robotPose.transformBy(RigidTransform2d.fromTranslation(new Translation2d(defaultX, defaultY)));
		currentX = modulePose.getTranslation().getX();
		currentY = modulePose.getTranslation().getY();
		lastDistance = 0;
	}
	@Override
	public void outputToSmartDashboard(){
		String smallX = Float.toString((float)(Math.round(getX() * 100.0) / 100.0));
		String smallY = Float.toString((float)(Math.round(getY() * 100.0) / 100.0));
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Angle", getModuleAngle());
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Goal", rotationMotor.getSetpoint());
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Position", getEncoderDistanceFeet());
		SmartDashboard.putString("Module " + Integer.toString(moduleID) + " Coordinates ", "("+smallX+" , "+smallY+")");
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + "Speed", getModuleFeetPerSecond());
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Error", rotationMotor.getSetpoint() - rotationMotor.getPosition());
		if(moduleID == 4){
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " X", currentX);
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Y", currentY);
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Field Relative Angle", getFieldRelativeAngle());
			
		}
	}
}
