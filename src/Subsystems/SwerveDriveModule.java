package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveModule extends Subsystem{
	static final double HALF_LENGTH = Constants.WHEELBASE_LENGTH/2;
	static final double HALF_WIDTH = Constants.WHEELBASE_WIDTH/2;
	
	private Pidgeon pidgey;
	
	private CANTalon rotationMotor;
	public CANTalon driveMotor;
	private int moduleID;
	private int absolutePosition;
	private double offset = 0.0;
	public double pathFollowingOffset = 0.0;
	private double currentDistance = 0.0;
	private double lastDistance = 0.0;
	private double currentX = 0;
	private double currentY = 0;
	public boolean hasBraked = false;
	public boolean hasBraked(){return hasBraked;}
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
    	rotationMotor.changeControlMode(TalonControlMode.Position);
    	rotationMotor.setPID(5.0, 0.0, 40.0, 0.00, 0, 0.0, 0);
    	rotationMotor.setProfile(0);
    	rotationMotor.set(rotationMotor.get());
    	driveMotor.setEncPosition(0);
    	driveMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	driveMotor.configEncoderCodesPerRev(360);
    	driveMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, 10);
    	driveMotor.configNominalOutputVoltage(+0f, -0f);
    	driveMotor.configPeakOutputVoltage(+12f, -12f);
    	driveMotor.setVoltageRampRate(0.0);
    	driveMotor.setAllowableClosedLoopErr(0);
    	driveMotor.changeControlMode(TalonControlMode.PercentVbus);
    	driveMotor.reverseOutput(false);
    	driveMotor.enableBrakeMode(true);
	}
	public double getRawAngle(){
		return rotationMotor.get();
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
		double newAngle = Util.continousAngle(goalAngle+offset,getRawAngle());
		rotationMotor.set(newAngle);
	}
	public double getGoal(){
		return Util.boundAngle0to360Degrees(rotationMotor.getSetpoint() - offset);
	}
	public void setDriveSpeed(double power){
		driveMotor.changeControlMode(TalonControlMode.PercentVbus);
		driveMotor.set(power);
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
		driveMotor.setProfile(0);
    	driveMotor.setPID(0.5, 0, 80.0, 0.10685189, 0, 0.0, 0);
    	driveMotor.setMotionMagicCruiseVelocity(3200);
    	driveMotor.setMotionMagicAcceleration(4500);
    	driveMotor.set(driveMotor.getPosition() + inchesToRotations(inches));
	}
	public boolean onDistanceTarget(){
		return (driveMotor.getControlMode() == TalonControlMode.MotionMagic) && 
				Math.abs(rotationsToInches(driveMotor.getSetpoint()) - rotationsToInches(driveMotor.getPosition())) < 2.0;
	}
	public void setOriginCoordinates(double x, double y){
		currentX = x;
		currentY = y;
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
		rotationMotor.set(rotationMotor.get());
		driveMotor.set(0);
	}
	@Override
	public synchronized void zeroSensors(){
		driveMotor.setEncPosition(0);
/*		currentX = Math.sin(90 - (180 - (pidgey.getAngle() + 90) + Math.toDegrees(Math.atan(HALF_WIDTH/HALF_LENGTH)))) * Math.hypot(HALF_LENGTH, HALF_WIDTH);
		currentY = Math.cos(90 - (180 - (pidgey.getAngle() + 90) + Math.toDegrees(Math.atan(HALF_WIDTH/HALF_LENGTH)))) * Math.hypot(HALF_LENGTH, HALF_WIDTH);
/**/	}
	@Override
	public void outputToSmartDashboard(){
		String smallX = Float.toString((float)(Math.round(getX() * 100.0) / 100.0));
		String smallY = Float.toString((float)(Math.round(getY() * 100.0) / 100.0));
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Angle", getModuleAngle());
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Goal", getGoal());
		SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Position", getEncoderDistanceFeet());
		SmartDashboard.putString("Module " + Integer.toString(moduleID) + " Coordinates ", "("+smallX+" , "+smallY+")");
		if(moduleID == 4){
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " X", currentX);
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Y", currentY);
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Field Relative Angle", getFieldRelativeAngle());
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + "Speed", getModuleFeetPerSecond());
		}
	}
}
