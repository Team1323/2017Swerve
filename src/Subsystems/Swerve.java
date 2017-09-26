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
	static final double HALF_LENGTH = Constants.WHEELBASE_LENGTH/2;
	static final double HALF_WIDTH = Constants.WHEELBASE_WIDTH/2;
	
	private static Swerve instance = null;
	private Pidgeon pidgey = null;
	private SwerveKinematics kinematics = new SwerveKinematics();
	private double xInput = 0;
	private double yInput = 0;
	private double inputMagnitude = 0;
	private double rotateInput = 0;
	private double rotationCorrection = 0;
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
	public boolean superSlow = false;
	public void toggleSuperSlow(){
		superSlow = !superSlow;
	}
	public boolean shouldBrake = true;
	
	public enum ControlState{
		Manual("Manual"), PathFollowing("PathFollowing"), Neutral("Neutral")
		, AdjustTargetDistance("AdjustTargetDistance"),
		TurnInPlace("TurnInPlace"), BaseLock("BaseLock");
		
		public final String name;
		private ControlState(String name){
			this.name = name;
		}
	}
	private ControlState currentState = ControlState.Manual;
	public ControlState getState(){
		return currentState;
	}
	public void setState(ControlState newState){
		currentState = newState;
	}
	double maxVel = 10.0;//13.89;
	double maxAccel = 10.0;//16;
	double maxJerk = 84;
	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.01, maxVel, maxAccel, maxJerk);
	Trajectory.Config stableConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, maxVel, maxAccel, maxJerk);
	Trajectory.Config gearConfig =  new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 3.0, maxAccel, maxJerk);
	SwerveModifier.Mode mode = SwerveModifier.Mode.SWERVE_DEFAULT;
	Trajectory redHopperTrajectory;
	Trajectory testTrajectory;
	Trajectory blueHopperTrajectory;
	Trajectory currentTrajectory;
	SwerveModifier modifier;
	DistanceFollower flFollower;
	DistanceFollower frFollower;
	DistanceFollower blFollower;
	DistanceFollower brFollower;
	
	public double dt;
	double timestamp;
	
	public enum Path{
		BLUE_HOPPER, RED_HOPPER, TEST, FORWARD, BACKWARD, GEAR
	}
	
	public enum HeadingController{
		Off("Off"), Stabilize("Stabilize"), Snap("Snap");
		
		public final String name;
		private HeadingController(String name){
			this.name = name;
		}
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
	private SynchronousPID snapPID = new SynchronousPID(0.0075, 0.0, 0.1, 0.2);
	private SynchronousPID reversibleSnapPID = new SynchronousPID(0.005, 0.0, 0.01, 0.0);
	private int cyclesLeft = 1;
	
	//305.2586 54.74135
	
	double robotX = 0.0;
	double robotY = 0.0;
	double distanceTraveled = 0.0;
	double distanceTraveledOffset = 0.0;
	public double getX(){
		return robotX;
	}
	public double getY(){
		return robotY;
	}
	public double getDistanceTraveled(){
		return distanceTraveled;
	}
	
	private boolean isManuallyRotating = false;
	private double manualRotationStopTime = 0.0;
	private boolean stabilizationTargetSet = false;
	
	public Swerve(){
		pidgey = Pidgeon.getInstance();
		
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2,Constants.FRONT_LEFT_TURN_OFFSET);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1,Constants.FRONT_RIGHT_TURN_OFFSET);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3,Constants.REAR_LEFT_TURN_OFFSET);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4,Constants.REAR_RIGHT_TURN_OFFSET);
		
		frontRight.driveMotor.reverseSensor(true);
		frontLeft.driveMotor.reverseOutput(true);
		rearLeft.driveMotor.reverseOutput(true);
		
		frontRight.setOriginCoordinates(HALF_WIDTH, HALF_LENGTH);
		frontLeft.setOriginCoordinates(-HALF_WIDTH, HALF_LENGTH);
		rearLeft.setOriginCoordinates(-HALF_WIDTH, -HALF_LENGTH);
		rearRight.setOriginCoordinates(HALF_WIDTH, -HALF_LENGTH);
		
		modules = new ArrayList<>(4);
		modules.add(frontRight);
		modules.add(frontLeft);
		modules.add(rearLeft);
		modules.add(rearRight);
		
		generatePaths();
		
		snapPID.setOutputRange(-0.5, 0.5);
		reversibleSnapPID.setOutputRange(-0.7, 0.7);
		strongHeadingPID.setOutputRange(-0.4, 0.4);
		
	}
	public static Swerve getInstance(){
		if(instance == null) instance = new Swerve();
        return instance;
    }
	
	private void generatePaths(){
		Waypoint[] stablePoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(-6, 0.5, Pathfinder.d2r(170)),
				new Waypoint(-7.5,2.5,Pathfinder.d2r(90)),
				new Waypoint(-5, 3.75, Pathfinder.d2r(0))
		};
		Waypoint[] bluePoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(-5.0, 1.85, Pathfinder.d2r(130)),
				new Waypoint(-5.35, 3.1, Pathfinder.d2r(50)),
				new Waypoint(-3.5, 3.09, Pathfinder.d2r(0))
		};
		Waypoint[] redPoints = new Waypoint[]{
				/*new Waypoint(0,0,0),
				new Waypoint(5.0, 1.35, Pathfinder.d2r(50)),
				new Waypoint(5.35, 2.6, Pathfinder.d2r(130)),
				new Waypoint(3.6, 2.58, Pathfinder.d2r(180))*/
				new Waypoint(0,0,0),
				new Waypoint(5.2, 1.85, Pathfinder.d2r(50)),
				new Waypoint(5.55, 3.1, Pathfinder.d2r(130)),
				new Waypoint(3.8, 3.09, Pathfinder.d2r(180))
		};
		Waypoint[] leftPegPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(8.0, 2.0, Pathfinder.d2r(60))
		};
		Waypoint[] leftPegToHopperPoints = new Waypoint[]{
				new Waypoint(0,0,-90),
				new Waypoint(-2.5, -6.75, Pathfinder.d2r(-130)),
				new Waypoint(-4.0, -6.75, Pathfinder.d2r(180))
		};
		
		
		//redHopperTrajectory = Pathfinder.generate(redPoints, stableConfig);
		//blueHopperTrajectory = Pathfinder.generate(bluePoints, stableConfig);
		testTrajectory = Pathfinder.generate(leftPegToHopperPoints, stableConfig);
		/*for (int i = 0; i < forwardTrajectory.length(); i++) {
		    Trajectory.Segment seg = forwardTrajectory.get(i);
		    Logger.log("(" + Double.toString(seg.y) + ", " + Double.toString(seg.x) + "), ");
		}*/
	}
	
	
	public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower){
		double angle = pidgey.getAngle()/180.0*Math.PI;
		inputMagnitude = Math.hypot(x, y);
		if(robotCentric){
			xInput = x;
			yInput = y;
		}
		else{
			double tmp = (y* Math.cos(angle)) + (x * Math.sin(angle));
			xInput = (-y * Math.sin(angle)) + (x * Math.cos(angle));
			yInput = tmp;			
		}
		if(superSlow){
			xInput *= 0.3;
			yInput *= 0.3;
		}else if(lowPower || forcedLowPower){
			xInput *= 0.45;
			yInput *= 0.45;
		}
		rotateInput = rotate*0.8;
		
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
		
		if(x != 0 || y != 0){
			shouldBrake = false;
		}else if(rotateInput != 0){
			shouldBrake = true;
		}
		
		if(xInput != 0 || yInput != 0 || rotateInput != 0){
			setState(ControlState.Manual);
		}
	}
	
	public void followPath(Path path, double heading){
		switch(path){
		case BLUE_HOPPER:
			modifier = new SwerveModifier(blueHopperTrajectory);
			currentTrajectory = blueHopperTrajectory;
			break;
		case RED_HOPPER:
			modifier = new SwerveModifier(redHopperTrajectory);
			currentTrajectory = redHopperTrajectory;
			break;
		case TEST:
			modifier = new SwerveModifier(testTrajectory);
			currentTrajectory = testTrajectory;
			break;
		case FORWARD:
			break;
		case BACKWARD:
			break;
		case GEAR:
			break;
		default:
			modifier = new SwerveModifier(blueHopperTrajectory);
			currentTrajectory = blueHopperTrajectory;
			break;
		}
		
		modifier.modify(Constants.WHEELBASE_WIDTH/12, Constants.WHEELBASE_LENGTH/12, mode);
		flFollower = new DistanceFollower(modifier.getFrontLeftTrajectory());
		frFollower = new DistanceFollower(modifier.getFrontRightTrajectory());
		blFollower = new DistanceFollower(modifier.getBackLeftTrajectory());
		brFollower = new DistanceFollower(modifier.getBackRightTrajectory());
		
		double p = 0.7;
		double d = 0.0;
		double v = 1/(maxVel);
		double a = 0.0;
		
		flFollower.configurePIDVA(p, 0.0, d, v, a);
		frFollower.configurePIDVA(p, 0.0, d, v, a);
		blFollower.configurePIDVA(p, 0.0, d, v, a);
		brFollower.configurePIDVA(p, 0.0, d, v, a);
		
		for(SwerveDriveModule m : modules){
			m.pathFollowingOffset = m.getEncoderDistanceFeet();
		}
		distanceTraveledOffset = distanceTraveled;
		
		setTargetHeading(heading);
		setHeadingController(HeadingController.Stabilize);
		setState(ControlState.PathFollowing);
	}
	
	public void followPath(Path path){
		followPath(path, pidgey.getAngle());
	}
	
	public boolean isFinishedWithPath(){
		return (getState() == ControlState.PathFollowing && 
				brFollower.getSegment().position >= (0.95 * currentTrajectory.get(currentTrajectory.length()-1).position));
	}
	public boolean strictFinishedWithPath(){
		return (getState() == ControlState.PathFollowing &&
				flFollower.isFinished() && frFollower.isFinished() &&
				blFollower.isFinished() && brFollower.isFinished());
	}
	
	public void moveDistance(double wheelAngle, double distance){
		setState(ControlState.AdjustTargetDistance);
		for(SwerveDriveModule m : modules){
			m.setFieldRelativeAngle(wheelAngle);
		}
		for(SwerveDriveModule m : modules){
			m.moveInches(distance);
		}
	}
	
	public void turnInPlace(double goalHeading){
		setState(ControlState.TurnInPlace);
		setTargetHeading(goalHeading);
		double deltaAngle = Math.toRadians(goalHeading - pidgey.getAngle());
		double swerveRadius = Math.hypot(HALF_LENGTH, HALF_WIDTH);
		double arcLength = deltaAngle * swerveRadius;
		double tangentAngle = Math.toDegrees(Math.atan2(Constants.WHEELBASE_LENGTH, Constants.WHEELBASE_WIDTH));
		frontRight.setModuleAngle(-tangentAngle);
		frontLeft.setModuleAngle(tangentAngle);
		rearLeft.setModuleAngle(-tangentAngle);
		rearRight.setModuleAngle(tangentAngle);
		frontRight.moveInches(-arcLength);
		frontLeft.moveInches(arcLength);
		rearLeft.moveInches(arcLength);
		rearRight.moveInches(-arcLength);
	}
	
	public boolean distanceOnTarget(){
		boolean onTarget = true;
		for(SwerveDriveModule m : modules){
			if(!m.onDistanceTarget()) onTarget = false;
		}
		return onTarget;
	}
	
	public void rotate(double targetHeading){
		if(inputMagnitude <= 0.1){
			turnInPlace(targetHeading);
		}else{
			setSnapAngle(targetHeading);
		}
	}
	
	public void baseLock(){
		setState(ControlState.BaseLock);
		frontRight.setModuleAngle(-Math.toDegrees(Math.atan2(Constants.WHEELBASE_LENGTH, Constants.WHEELBASE_WIDTH))-90);
		frontLeft.setModuleAngle(Math.toDegrees(Math.atan2(Constants.WHEELBASE_LENGTH, Constants.WHEELBASE_WIDTH))+90);
		rearLeft.setModuleAngle(Math.toDegrees(Math.atan2(Constants.WHEELBASE_WIDTH, Constants.WHEELBASE_LENGTH)));
		rearRight.setModuleAngle(-Math.toDegrees(Math.atan2(Constants.WHEELBASE_WIDTH, Constants.WHEELBASE_LENGTH)));
		for(SwerveDriveModule m : modules){
			m.lockPosition();
		}
	}
	
	private final Loop swerveLoop = new Loop(){
		@Override
		public void onStart(){
			stop();
		}
		@Override
		public void onLoop(){
			update();
			double x = 0;
			double y = 0;
			for(SwerveDriveModule m : modules){
				m.update();
				x += m.getX();
				y += m.getY();
			}
			distanceTraveled += Math.hypot(robotX - (x/4), robotY - (y/4));
			robotX = x/4;
			robotY = y/4;
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
		double now = Timer.getFPGATimestamp();
		rotationCorrection = 0;
		switch(headingController){
			case Off:
				targetHeadingAngle = pidgey.getAngle();
				break;
			case Stabilize:
				if(!stabilizationTargetSet && (Timer.getFPGATimestamp() - manualRotationStopTime >= 0.45)){
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
				break;
			case Snap:
				//rotationCorrection = snapPID.calculate(getHeadingError());
				rotationCorrection = reversibleSnapPID.calculate(getHeadingError());
				if(Math.abs(getHeadingError()) < 2){
					cyclesLeft--;
				}else{
					cyclesLeft = 1;
				}
				if(cyclesLeft <= 0){
					setHeadingController(HeadingController.Stabilize);
					rotationCorrection = 0;
				}
				break;
			default:
				break;
		}
		
		rotateInput += rotationCorrection;
		
		switch(currentState){
			case Manual:
				kinematics.calculate(xInput, yInput, rotateInput);
			    if(xInput == 0 && yInput == 0 && Math.abs(rotateInput) <= 0.1){
			    	if(shouldBrake){
					    for(SwerveDriveModule m : modules){
					    	if(!m.hasBraked()){
					    		m.setModuleAngle(m.getModuleAngle() + 90);
					    		m.hasBraked = true;
					    	}
					    }
			    	}
			    }else{
			    	for(SwerveDriveModule m : modules){
			    		m.hasBraked = false;
			    	}
			    	for(int i=0; i<4; i++){
			    		if(Util.shouldReverse(kinematics.wheelAngles[i], modules.get(i).getModuleAngle())){
			    			modules.get(i).setModuleAngle(kinematics.wheelAngles[i] + 180);
			    		}else{
			    			modules.get(i).setModuleAngle(kinematics.wheelAngles[i]);
			    		}
			    	}
			    	/*if(Util.shouldReverse(kinematics.frSteeringAngle(), frontRight.getModuleAngle())){
			    		frontRight.setModuleAngle(kinematics.frSteeringAngle() + 180);
			    		System.out.println("Is reversing");
			    	}else{
			    		frontRight.setModuleAngle(kinematics.frSteeringAngle());
			    	}*/
				    /*frontRight.setModuleAngle(kinematics.frSteeringAngle());
				    frontLeft.setModuleAngle(kinematics.flSteeringAngle());
				    rearLeft.setModuleAngle(kinematics.rlSteeringAngle());
				    rearRight.setModuleAngle(kinematics.rrSteeringAngle());*/
			    }
			    for(int i=0; i<4; i++){
		    		if(Util.shouldReverse(kinematics.wheelAngles[i], modules.get(i).getModuleAngle())){
		    			if(i == 1 || i == 2){
		    				modules.get(i).setDriveSpeed(kinematics.wheelSpeeds[i]);
		    			}else{
		    				modules.get(i).setDriveSpeed(-kinematics.wheelSpeeds[i]);
		    			}
		    		}else{
		    			if(i == 1 || i == 2){
		    				modules.get(i).setDriveSpeed(-kinematics.wheelSpeeds[i]);
		    			}else{
		    				modules.get(i).setDriveSpeed(kinematics.wheelSpeeds[i]);
		    			}
		    		}
			    }
			    
			    /*frontRight.setDriveSpeed(kinematics.frWheelSpeed());
			    frontLeft.setDriveSpeed(-kinematics.flWheelSpeed());
			    rearLeft.setDriveSpeed(-kinematics.rlWheelSpeed());
			    rearRight.setDriveSpeed(kinematics.rrWheelSpeed());*/
				break;
			case PathFollowing:
/*/
				double fro = frFollower.calculate((frontRight.getEncoderDistanceFeet() - frontRight.pathFollowingOffset));
			    double flo = flFollower.calculate((rearLeft.getEncoderDistanceFeet() - rearLeft.pathFollowingOffset));
			    double blo = blFollower.calculate((rearLeft.getEncoderDistanceFeet() - rearLeft.pathFollowingOffset));
			    double bro = brFollower.calculate((rearRight.getEncoderDistanceFeet() - rearRight.pathFollowingOffset));
/*/				double fro = frFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
			    double flo = flFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
			    double blo = blFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
			    double bro = brFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
/**/
			    double pathWheelAngle = (frFollower.getHeading() + flFollower.getHeading() + 
			    		blFollower.getHeading() + brFollower.getHeading())/4;
			    double pidgeyAngle = Math.toRadians(pidgey.getAngle());
			    double x = Math.sin(pathWheelAngle);
			    double y = Math.cos(pathWheelAngle);
			    double tmp = (y* Math.cos(pidgeyAngle)) + (x * Math.sin(pidgeyAngle));
				xInput = (-y * Math.sin(pidgeyAngle)) + (x * Math.cos(pidgeyAngle));
				yInput = tmp;	
			    kinematics.calculate(xInput, yInput, rotationCorrection);
			    
				/*frontRight.setModuleAngle(Util.boundAngle0to360Degrees(Math.toDegrees(frFollower.getHeading())));
				frontLeft.setModuleAngle(Util.boundAngle0to360Degrees(Math.toDegrees(flFollower.getHeading())));
				rearLeft.setModuleAngle(Util.boundAngle0to360Degrees(Math.toDegrees(blFollower.getHeading())));
				rearRight.setModuleAngle(Util.boundAngle0to360Degrees(Math.toDegrees(brFollower.getHeading())));*/
			    
/**/
			    frontRight.setModuleAngle(kinematics.frSteeringAngle());
			    frontLeft.setModuleAngle(kinematics.flSteeringAngle());
			    rearLeft.setModuleAngle(kinematics.rlSteeringAngle());
			    rearRight.setModuleAngle(kinematics.rrSteeringAngle());
/*/
			    
			    frontRight.setFieldRelativeAngle(kinematics.frSteeringAngle());
			    frontLeft.setFieldRelativeAngle(kinematics.flSteeringAngle());
			    rearLeft.setFieldRelativeAngle(kinematics.rlSteeringAngle());
			    rearRight.setFieldRelativeAngle(kinematics.rrSteeringAngle());
/**/
				frontRight.setDriveSpeed(fro);
				frontLeft.setDriveSpeed(-flo);
				rearRight.setDriveSpeed(bro);
				rearLeft.setDriveSpeed(-blo);
				
				if(brFollower.isFinished()){
					setState(ControlState.Neutral);
				}
				
				break;
			case AdjustTargetDistance:
				
				break;
			case TurnInPlace:
				if(distanceOnTarget()){
					turnInPlace(targetHeadingAngle);
				}
				break;
			case BaseLock:
				
				break;
			case Neutral:
				stop();
				break;
		}
		dt = now - timestamp;
		timestamp = now;
	}
	public void setModuleAngles(double angle){
		for(SwerveDriveModule m : modules){
			m.setModuleAngle(angle);
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
		SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
		SmartDashboard.putNumber("Rotation Correction", rotationCorrection);
		SmartDashboard.putString("Swerve Control State", currentState.name);
		SmartDashboard.putString("Heading Controller", headingController.name);
	}
	
}