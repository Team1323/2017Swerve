package Subsystems;

import java.util.ArrayList;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Logger;
import Utilities.Ports;
import Utilities.SynchronousPID;
import Utilities.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.SwerveModifier;

public class Swerve extends Subsystem{
	private static Swerve instance = new Swerve();
	private Pidgeon pidgey = Pidgeon.getInstance();
	private SwerveKinematics kinematics = new SwerveKinematics();
	private double xInput;
	private double yInput;
	private double rotateInput;
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	public SwerveDriveModule rearRight;
	ArrayList<SwerveDriveModule> modules;
	
	double encoderOffset = 0.0;
	
	private boolean forcedLowPower = false;
	public void setLowPower(boolean on){
		forcedLowPower = on;
	}
	
	public enum ControlState{
		Manual, PathFollowing
	}
	private ControlState currentState = ControlState.Manual;
	public ControlState getState(){
		return currentState;
	}
	public void setState(ControlState newState){
		currentState = newState;
	}
	double maxVel = 13.89;
	double maxAccel = 16;
	double maxJerk = 84;
	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.01, maxVel, maxAccel, maxJerk);
	SwerveModifier.Mode mode = SwerveModifier.Mode.SWERVE_DEFAULT;
	Trajectory leftTrajectory;
	Trajectory rightTrajectory;
	SwerveModifier modifier;
	DistanceFollower flFollower;
	DistanceFollower frFollower;
	DistanceFollower blFollower;
	DistanceFollower brFollower;
	
	public enum HeadingController{
		Off, Stabilize, Snap
	}
	private HeadingController headingController = HeadingController.Stabilize;
	public void setHeadingController(HeadingController h){
		headingController = h;
	}
	private double targetHeadingAngle = 0.0;
	public void setTargetHeading(double target){
		targetHeadingAngle = target;
		stabilizationTargetSet = true;
	}
	public void setSnapAngle(double target){
		targetHeadingAngle = target;
		setHeadingController(HeadingController.Snap);
	}
	private SynchronousPID headingPID = new SynchronousPID(0.001, 0.0, 0.0007, 0.0);
	private SynchronousPID strongHeadingPID = new SynchronousPID(0.008, 0.0, 0.0, 0.0);
	private SynchronousPID snapPID = new SynchronousPID(0.01, 0.0, 0.1, 0.2);
	private int cyclesLeft = 1;
	
	static final double HALF_LENGTH = Constants.WHEELBASE_LENGTH/2;
	static final double HALF_WIDTH = Constants.WHEELBASE_WIDTH/2;
	double robotX = 0.0;
	double robotY = 0.0;
	public double getX(){
		return robotX;
	}
	public double getY(){
		return robotY;
	}
	
	private boolean isManuallyRotating = false;
	private double manualRotationStopTime = 0.0;
	private boolean stabilizationTargetSet = false;
	
	public Swerve(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2,Constants.FRONT_LEFT_TURN_OFFSET);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1,Constants.FRONT_RIGHT_TURN_OFFSET);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3,Constants.REAR_LEFT_TURN_OFFSET);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4,Constants.REAR_RIGHT_TURN_OFFSET);
		
		modules = new ArrayList<>(4);
		modules.add(frontLeft);
		modules.add(frontRight);
		modules.add(rearLeft);
		modules.add(rearRight);
		
		snapPID.setOutputRange(-0.5, 0.5);
		
		Waypoint[] rightPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(-6, 0.5, Pathfinder.d2r(175)),
				new Waypoint(-7,2.5,Pathfinder.d2r(90)),
				new Waypoint(-5, 3.5, Pathfinder.d2r(0))
		};
		Waypoint[] leftPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(-8,-8,0),
		};
		//leftTrajectory = Pathfinder.generate(leftPoints, config);
		rightTrajectory = Pathfinder.generate(rightPoints, config);
		/*for (int i = 0; i < rightTrajectory.length(); i++) {
		    Trajectory.Segment seg = rightTrajectory.get(i);
		    Logger.log("x: " + Double.toString(seg.x) + ", y: " + Double.toString(seg.y) + ", v: " + Double.toString(seg.velocity) + ", a: " + Double.toString(seg.acceleration) + ", j: " + Double.toString(seg.jerk) + ", h: " + Double.toString(seg.heading) + "\n");
		}*/
		for (int i = 0; i < rightTrajectory.length(); i++) {
		    Trajectory.Segment seg = rightTrajectory.get(i);
		    /**/
		    Logger.log("(" + Double.toString(seg.y) + ", " + Double.toString(seg.x) + "), ");
		    /*/
		    Logger.log(Double.toString(seg.velocity) + ", " + Double.toString(seg.acceleration));
		    /**/
		}
	}
	public static Swerve getInstance(){
        return instance;
    }
	
	public class SwerveDriveModule extends Subsystem{
		private CANTalon rotationMotor;
		public CANTalon driveMotor;
		private int moduleID;
		private int absolutePosition;
		private double offSet = 0.0;
		private double currentDistance = 0.0;
		private double lastDistance = 0.0;
		private double currentX = HALF_WIDTH;
		private double currentY = -HALF_LENGTH;
		public SwerveDriveModule(int rotationMotorPort, int driveMotorPort,int moduleNum,double _offSet){
			rotationMotor = new CANTalon(rotationMotorPort);
			driveMotor = new CANTalon(driveMotorPort);
			moduleID = moduleNum;  
			offSet = _offSet;
			loadProperties();
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
			return Util.boundAngle0to360Degrees(getRawAngle() - offSet);
		}
		public double getFieldRelativeAngle(){
			return Util.boundAngle0to360Degrees(getModuleAngle() + Util.boundAngle0to360Degrees(pidgey.getAngle()));
		}
		public void setFieldRelativeAngle(double fieldAngle){
			setModuleAngle(Util.boundAngle0to360Degrees(fieldAngle - Util.boundAngle0to360Degrees(pidgey.getAngle())));
		}
		public void setModuleAngle(double goalAngle){
			double newAngle = Util.continousAngle(goalAngle-(360-offSet),getRawAngle());
			rotationMotor.set(newAngle);
		}
		public double getGoal(){
			return Util.boundAngle0to360Degrees(rotationMotor.getSetpoint() - offSet);
		}
		public void setDriveSpeed(double power){
			driveMotor.set(power);
		}
		public double getEncoderDistanceInches(){
			return driveMotor.getPosition()/Constants.SWERVE_ENCODER_REVS_PER_INCH;
		}
		public double getEncoderDistanceFeet(){
			return getEncoderDistanceInches()/12;
		}
		public double getModuleInchesPerSecond(){
			return driveMotor.getSpeed()/Constants.SWERVE_ENCODER_REVS_PER_INCH/60;
		}
		public double getModuleFeetPerSecond(){
			return getModuleInchesPerSecond()/12;
		}
		public void update(){
			currentDistance = getEncoderDistanceInches();
			double distanceTraveled = currentDistance - lastDistance;
			double angle = Math.toRadians(90 - getFieldRelativeAngle());
			currentX += Math.cos(angle) * distanceTraveled;
			currentY += Math.sin(angle) * distanceTraveled;
			lastDistance = currentDistance;
			
			double halfDiagonal = Math.toDegrees(Math.atan(HALF_WIDTH/HALF_LENGTH));
			double robotAngle = Math.toRadians(180 - ((180-Util.boundAngle0to360Degrees(pidgey.getAngle()))+halfDiagonal));
			robotX = currentX + (Math.sin(robotAngle) * Math.hypot(HALF_LENGTH, HALF_WIDTH));
			robotY = currentY + (Math.cos(robotAngle) * Math.hypot(HALF_LENGTH, HALF_WIDTH));
			
		}
		@Override
		public synchronized void stop(){
			rotationMotor.set(rotationMotor.get());
			driveMotor.set(0);
		}
		@Override
		public synchronized void zeroSensors(){
			driveMotor.setEncPosition(0);
			currentX = Math.sin(90 - (180 - (pidgey.getAngle() + 90) + Math.toDegrees(Math.atan(HALF_WIDTH/HALF_LENGTH)))) * Math.hypot(HALF_LENGTH, HALF_WIDTH);
			currentY = Math.cos(90 - (180 - (pidgey.getAngle() + 90) + Math.toDegrees(Math.atan(HALF_WIDTH/HALF_LENGTH)))) * Math.hypot(HALF_LENGTH, HALF_WIDTH);
		}
		@Override
		public void outputToSmartDashboard(){
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Angle", getModuleAngle());
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Goal", getGoal());
			if(moduleID == 4){
				SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Enc Position", driveMotor.getEncPosition());
				SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Position", getEncoderDistanceFeet());
				SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " X", currentX);
				SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Y", currentY);
				SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Field Relative Angle", getFieldRelativeAngle());
				SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + "Speed", getModuleFeetPerSecond());
			}
		}
	}
	
	public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower){
		double angle = pidgey.getAngle()/180.0*Math.PI;
		if(robotCentric){
			xInput = x;
			yInput = y;
		}
		else{
			double tmp = (y* Math.cos(angle)) + (x * Math.sin(angle));
			xInput = (-y * Math.sin(angle)) + (x * Math.cos(angle));
			yInput = tmp;			
		}
		if(lowPower || forcedLowPower){
			xInput *= 0.45;
			yInput *= 0.45;
		}
		rotateInput = rotate*0.6;
		
		if(rotateInput == 0){
			if(isManuallyRotating){
				isManuallyRotating = false;
				stabilizationTargetSet = false;
				manualRotationStopTime = Timer.getFPGATimestamp();
			}
			if(headingController == HeadingController.Off){
				setHeadingController(HeadingController.Stabilize);
			}
		}else{
			setHeadingController(HeadingController.Off);
			isManuallyRotating = true;
		}
	}
	
	public void followPath(boolean left){
		if(left){
			modifier = new SwerveModifier(leftTrajectory);
		}else{
			modifier = new SwerveModifier(rightTrajectory);
		}
		
		modifier.modify(Constants.WHEELBASE_WIDTH/12, Constants.WHEELBASE_LENGTH/12, mode);
		flFollower = new DistanceFollower(modifier.getFrontLeftTrajectory());
		frFollower = new DistanceFollower(modifier.getFrontRightTrajectory());
		blFollower = new DistanceFollower(modifier.getBackLeftTrajectory());
		brFollower = new DistanceFollower(modifier.getBackRightTrajectory());
		
		double d = 0.0;
		double p = 1.0;
		double a = 0.0;
		double v = 1/(maxVel);
		
		flFollower.configurePIDVA(p, 0.0, d, v, a);
		frFollower.configurePIDVA(p, 0.0, d, v, a);
		blFollower.configurePIDVA(p, 0.0, d, v, a);
		brFollower.configurePIDVA(p, 0.0, d, v, a);
		
		encoderOffset = rearRight.getEncoderDistanceFeet();
		
		setState(ControlState.PathFollowing);
	}
	
	private final Loop swerveLoop = new Loop(){
		@Override
		public void onStart(){
			stop();
		}
		@Override
		public void onLoop(){
			update();
			rearRight.update();
		}
		@Override
		public void onStop(){
			stop();
		}
	};
	public Loop getLoop(){
		return swerveLoop;
	}
	
	public double getHeadingError(){
		return pidgey.getAngle() - targetHeadingAngle;
	}
	public void update(){
		switch(currentState){
			case Manual:
				double rotationCorrection = 0.0;
				switch(headingController){
					case Off:
						targetHeadingAngle = pidgey.getAngle();
						SmartDashboard.putString("Heading Controller", "Off");
						break;
					case Stabilize:
						if(!stabilizationTargetSet && (Timer.getFPGATimestamp() - manualRotationStopTime >= 0.3)){
							targetHeadingAngle = pidgey.getAngle();
							stabilizationTargetSet = true;
						}
						if(stabilizationTargetSet){
							if(Math.abs(getHeadingError()) > 5){
								rotationCorrection = strongHeadingPID.calculate(getHeadingError());
							}else if(Math.abs(getHeadingError()) > 0.5){
								rotationCorrection = headingPID.calculate(getHeadingError());
							}
						}
						SmartDashboard.putString("Heading Controller", "Stabilize");
						break;
					case Snap:
						rotationCorrection = snapPID.calculate(getHeadingError());
						if(Math.abs(getHeadingError()) < 2){
							cyclesLeft--;
						}else{
							cyclesLeft = 1;
						}
						if(cyclesLeft <= 0){
							setHeadingController(HeadingController.Stabilize);
							rotationCorrection = 0;
						}
						SmartDashboard.putString("Heading Controller", "Snap");
						break;
					default:
						break;
				}
				SmartDashboard.putNumber("Rotation Correction", rotationCorrection);
				rotateInput += rotationCorrection;
				
				kinematics.calculate(xInput, yInput, rotateInput);
			    if(xInput == 0 && yInput == 0 && Math.abs(rotateInput) <= 0.01){
				    /*for(SwerveDriveModule m : modules){
				    	m.setModuleAngle(0);
				    }*/
			    	frontRight.setModuleAngle(45);
			    	frontLeft.setModuleAngle(-45);
			    	rearLeft.setModuleAngle(-135);
			    	rearRight.setModuleAngle(135);
			    }else{
				    frontRight.setModuleAngle(kinematics.frSteeringAngle());
				    frontLeft.setModuleAngle(kinematics.flSteeringAngle());
				    rearLeft.setModuleAngle(kinematics.rlSteeringAngle());
				    rearRight.setModuleAngle(kinematics.rrSteeringAngle());
			    }
			    
			    frontRight.setDriveSpeed(kinematics.frWheelSpeed());
			    frontLeft.setDriveSpeed(-kinematics.flWheelSpeed());
			    rearLeft.setDriveSpeed(-kinematics.rlWheelSpeed());
			    rearRight.setDriveSpeed(kinematics.rrWheelSpeed());
				break;
			case PathFollowing:
				double fro = frFollower.calculate((rearRight.getEncoderDistanceFeet() - encoderOffset));
			    double flo = flFollower.calculate((rearRight.getEncoderDistanceFeet() - encoderOffset));
			    double blo = blFollower.calculate((rearRight.getEncoderDistanceFeet() - encoderOffset));
			    double bro = brFollower.calculate((rearRight.getEncoderDistanceFeet() - encoderOffset));
			    
				frontRight.setFieldRelativeAngle(Util.boundAngle0to360Degrees(Math.toDegrees(frFollower.getHeading())));
				frontLeft.setFieldRelativeAngle(Util.boundAngle0to360Degrees(Math.toDegrees(flFollower.getHeading())));
				rearLeft.setFieldRelativeAngle(Util.boundAngle0to360Degrees(Math.toDegrees(blFollower.getHeading())));
				rearRight.setFieldRelativeAngle(Util.boundAngle0to360Degrees(Math.toDegrees(brFollower.getHeading())));
				
				frontRight.setDriveSpeed(fro);
				frontLeft.setDriveSpeed(-flo);
				rearRight.setDriveSpeed(bro);
				rearLeft.setDriveSpeed(-blo);
				
				if(brFollower.isFinished()){
					setState(ControlState.Manual);
				}
				break;
		}
	}
	@Override
	public synchronized void stop(){
		sendInput(0.0, 0.0, 0.0, false, false);
		for(SwerveDriveModule m : modules){
			m.stop();
		}
	}
	@Override
	public synchronized void zeroSensors(){
		for(SwerveDriveModule m : modules){
			m.zeroSensors();
		}
		robotX = 0;
		robotY = 0;
	}
	@Override
	public void outputToSmartDashboard(){
		for(SwerveDriveModule m : modules){
			m.outputToSmartDashboard();
		}
		SmartDashboard.putNumber("Target Heading", targetHeadingAngle);
		SmartDashboard.putNumber("Robot X", robotX);
		SmartDashboard.putNumber("Robot Y", robotY);
	}
	
}