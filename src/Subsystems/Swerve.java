package Subsystems;

import java.util.ArrayList;

import com.team254.lib.util.control.Lookahead;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.control.PathFollower;

import Loops.Loop;
import Utilities.Constants;
import Utilities.DriveSignal;
import Utilities.Ports;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.SynchronousPIDF;
import Utilities.Translation2d;
import Utilities.Twist2d;
import Utilities.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
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
	private double desiredWheelHeading = 0;
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	public SwerveDriveModule rearRight;
	ArrayList<SwerveDriveModule> modules;
	
	SwerveDriveModule centerOfRotationModule;
	SwerveDriveModule lengthwiseModule;
	SwerveDriveModule widthwiseModule;
	SwerveDriveModule oppositeModule;
	
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
		TurnInPlace("TurnInPlace"), BaseLock("BaseLock"), 
		ModuleRotation("Clockwise"), Tank("TankDrive"),
		PurePursuit("PurePursuit");
		
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
	
	PathFollower mPathFollower;
	Path currentPath;
	
	double maxVel = 10.0;//13.89;
	double maxAccel = 6.0;//16;
	double maxJerk = 84;
	Trajectory.Config tolerantConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 0.02, 12.0, maxAccel, maxJerk);
	Trajectory.Config stableConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, maxVel, maxAccel, maxJerk);
	Trajectory.Config middlePegConfig =  new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 0.02, 6.0, maxAccel, maxJerk);
	Trajectory.Config sidePegConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 8.0, maxAccel, maxJerk);
	SwerveModifier.Mode mode = SwerveModifier.Mode.SWERVE_DEFAULT;
	public Trajectory redHopperTrajectory;
	public Trajectory testTrajectory;
	public Trajectory blueHopperTrajectory;
	public Trajectory leftPegTrajectory;
	public Trajectory leftPegToHopperTrajectory;
	public Trajectory rightPegTrajectory;
	public Trajectory rightPegToHopperTrajectory;
	public Trajectory middleGearTrajectory;
	public Trajectory middleToBlueBoilerTrajectory;
	public Trajectory middleToRedBoilerTrajectory;
	Trajectory currentTrajectory;
	SwerveModifier modifier;
	DistanceFollower flFollower;
	DistanceFollower frFollower;
	DistanceFollower blFollower;
	DistanceFollower brFollower;
	
	int currentSegment = 0;
	public double dt;
	double timestamp;
	
	public enum PathfinderPath{
		BLUE_HOPPER, RED_HOPPER, TEST, LEFT_PEG, LEFT_PEG_TO_HOPPER, MIDDLE_GEAR
	}
	
	public enum HeadingController{
		Off("Off"), Stabilize("Stabilize"), Snap("Snap"),
		TemporaryDisable("TemporaryDisable");
		
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
	public void temporarilyDisable(){
		targetHeadingAngle = pidgey.getAngle();
		disabledTimestamp = Timer.getFPGATimestamp();
		setHeadingController(HeadingController.TemporaryDisable);
	}
	private SynchronousPIDF headingPID = new SynchronousPIDF/*(0.002, 0.0, 0.0007, 0.0);*/(0.001, 0.0, 0.0007, 0.0);
	private SynchronousPIDF strongHeadingPID = new SynchronousPIDF(0.016, 0.0, 0.16, 0.0);/*(0.008, 0.0, 0.0, 0.0);*/
	private SynchronousPIDF snapPID = new SynchronousPIDF(0.0075, 0.0, 0.1, 0.2);
	private SynchronousPIDF reversibleSnapPID = new SynchronousPIDF(0.005, 0.0, 0.02, 0.0);
	private int cyclesLeft = 1;
	private double disabledTimestamp = 0.0;
	
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
		
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1,Constants.FRONT_RIGHT_TURN_OFFSET);
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2,Constants.FRONT_LEFT_TURN_OFFSET);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3,Constants.REAR_LEFT_TURN_OFFSET);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4,Constants.REAR_RIGHT_TURN_OFFSET);
		
		frontRight.driveMotor.setInverted(false);
		frontLeft.driveMotor.setInverted(false);
		rearLeft.driveMotor.setInverted(false);
		
		frontLeft.reverseOpenLoop(true);
		rearLeft.reverseOpenLoop(true);
		
		frontRight.setOriginCoordinates(HALF_LENGTH, -HALF_WIDTH);
		frontLeft.setOriginCoordinates(HALF_LENGTH, HALF_WIDTH);
		rearLeft.setOriginCoordinates(-HALF_LENGTH, HALF_WIDTH);
		rearRight.setOriginCoordinates(-HALF_LENGTH, -HALF_WIDTH);
		
		modules = new ArrayList<>(4);
		modules.add(frontRight);
		modules.add(frontLeft);
		modules.add(rearLeft);
		modules.add(rearRight);
		
		centerOfRotationModule = rearLeft;
		
		generatePaths();
		
		snapPID.setOutputRange(-0.5, 0.5);
		reversibleSnapPID.setOutputRange(-0.7, 0.7);
		strongHeadingPID.setOutputRange(-0.4, 0.4);
		
	}
	public static Swerve getInstance(){
		if(instance == null) instance = new Swerve();
        return instance;
    }
	
	public void generatePaths(){
		Waypoint[] bluePoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(5.0, -1.85, Pathfinder.d2r(130)),
				new Waypoint(5.35, -3.1, Pathfinder.d2r(50)),
				new Waypoint(3.5, -3.09, Pathfinder.d2r(0))
		};
		Waypoint[] redPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(5.2, 1.85, Pathfinder.d2r(50)),
				new Waypoint(5.55, 3.1, Pathfinder.d2r(130)),
				new Waypoint(3.8, 3.09, Pathfinder.d2r(180))
		};
		Waypoint[] straightGearPath = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(75.0/12.0, 0.0, Pathfinder.d2r(0))
		};
		Waypoint[] middleToBlueBoilerPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(-1.5, 0, Pathfinder.d2r(-135)),
				new Waypoint(-4, -6.5, Pathfinder.d2r(-90))
		};
		Waypoint[] middleToRedBoilerPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(-1.5, 0, Pathfinder.d2r(135)),
				new Waypoint(-6.0, 5.5, Pathfinder.d2r(90))
		};
		/*Waypoint[] leftPegPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(7.95, 2.0, Pathfinder.d2r(60))
		};*/
		double straightDistance = 1.25;
		double pegForwardDistance = 8.1;
		double pegSideDistance = 2.2;
		Waypoint[] leftPegPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(pegForwardDistance - (straightDistance/2), pegSideDistance - (straightDistance/2*Math.sqrt(3)), Pathfinder.d2r(60)),
				new Waypoint(pegForwardDistance, pegSideDistance, Pathfinder.d2r(60))
		};
		Waypoint[] leftPegToHopperPoints = new Waypoint[]{
				new Waypoint(0,0,Pathfinder.d2r(-90)),
				new Waypoint(-2.5, -7.75, Pathfinder.d2r(-130)),
				new Waypoint(-4.0, -7.5, Pathfinder.d2r(180))
		};
		double redStraightDistance = 1.25;
		double redForwardDistance = 7.9;
		double redSideDistance = 2.2;
		Waypoint[] rightPegPoints = new Waypoint[]{
				new Waypoint(0,0,0),
				new Waypoint(redForwardDistance - (redStraightDistance/2), -redSideDistance + (redStraightDistance/2*Math.sqrt(3)), Pathfinder.d2r(-60)),
				new Waypoint(redForwardDistance, -redSideDistance, Pathfinder.d2r(-60))
		};
		Waypoint[] rightPegToHopperPoints = new Waypoint[]{
				new Waypoint(0,0,Pathfinder.d2r(90)),
				new Waypoint(-1.5, 7.5, Pathfinder.d2r(130)),
				new Waypoint(-3.0, 7.25, Pathfinder.d2r(180))
		};
		
		Waypoint[] rightCubeDropoffPoints = new Waypoint[]{
				new Waypoint(0,0,Pathfinder.d2r(50)),
				new Waypoint(10.0, 7.5, 0),
				//new Waypoint(16.25, 7.5, 0),
				//new Waypoint(18.75, 5.0, Pathfinder.d2r(-45))
		};
		
		Waypoint[] leftCubeDropoffPoints = new Waypoint[]{
				new Waypoint(0,0,Pathfinder.d2r(-50)),
				new Waypoint(10.0, -7.5, 0),
				//new Waypoint(16.25, -7.5, 0),
				//new Waypoint(18.75, -5.0, Pathfinder.d2r(45))
		};
		
		testTrajectory = Pathfinder.generate(rightCubeDropoffPoints, tolerantConfig);
		/*redHopperTrajectory = Pathfinder.generate(redPoints, tolerantConfig);
		blueHopperTrajectory = Pathfinder.generate(bluePoints, tolerantConfig);
		leftPegTrajectory = Pathfinder.generate(leftPegPoints, sidePegConfig);
		leftPegToHopperTrajectory = Pathfinder.generate(leftPegToHopperPoints, tolerantConfig);
		rightPegTrajectory = Pathfinder.generate(rightPegPoints, sidePegConfig);
		rightPegToHopperTrajectory = Pathfinder.generate(rightPegToHopperPoints, tolerantConfig); 
		middleGearTrajectory = Pathfinder.generate(straightGearPath, middlePegConfig);
		middleToBlueBoilerTrajectory = Pathfinder.generate(middleToBlueBoilerPoints, tolerantConfig);
		middleToRedBoilerTrajectory = Pathfinder.generate(middleToRedBoilerPoints, tolerantConfig);*/
		/*for (int i = 0; i < testTrajectory.length(); i++) {
		    Trajectory.Segment seg = testTrajectory.get(i);
		    String coordinates = "(" + Double.toString(seg.y) + ", " + Double.toString(seg.x) + ")";
		    if(i != (testTrajectory.length() - 1))
		    	coordinates += ", ";
		    Logger.log(coordinates);
		}*/
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
		if(superSlow){
			xInput *= 0.3;
			yInput *= 0.3;
		}else if(lowPower || forcedLowPower){
			xInput *= 0.45;
			yInput *= 0.45;
		}
		inputMagnitude = Math.hypot(xInput, yInput);
		rotateInput = rotate*0.8;
		/*double theta = Math.atan2(yInput, xInput);
		double rotationOffset = Math.PI/2*rotateInput/(inputMagnitude*2);
		theta += rotationOffset;
		xInput = Math.cos(theta)*inputMagnitude;
		yInput = Math.sin(theta)*inputMagnitude;*/
		
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
			setState(ControlState.Manual);
		}else if(rotateInput != 0){
			if(Math.abs(frontRight.getModuleInchesPerSecond()) < 24){
				shouldBrake = false;
			}else{
				shouldBrake = true;
			}
			if(currentState != ControlState.ModuleRotation){
				setState(ControlState.Manual);
			}
		}
	}
	
	public void followPath(Trajectory trajectory, double heading){
		zeroSensors();
		
		modifier = new SwerveModifier(trajectory);
		currentTrajectory = trajectory;
		
		modifier.modify(Constants.WHEELBASE_WIDTH/12, Constants.WHEELBASE_LENGTH/12, mode);
		flFollower = new DistanceFollower(modifier.getFrontLeftTrajectory());
		frFollower = new DistanceFollower(modifier.getFrontRightTrajectory());
		blFollower = new DistanceFollower(modifier.getBackLeftTrajectory());
		brFollower = new DistanceFollower(modifier.getBackRightTrajectory());
		
		double p = 1.0;
		double d = 0.0;
		double v = 1.0/(13.89);
		double a = 0.0;
		
		flFollower.configurePIDVA(p, 0.0, d, v, a);
		frFollower.configurePIDVA(p, 0.0, d, v, a);
		blFollower.configurePIDVA(p, 0.0, d, v, a);
		brFollower.configurePIDVA(p, 0.0, d, v, a);
		
		for(SwerveDriveModule m : modules){
			m.pathFollowingOffset = m.getEncoderDistanceFeet();
		}
		distanceTraveledOffset = distanceTraveled;
		currentSegment = 0;
		
		setTargetHeading(heading);
		setHeadingController(HeadingController.Stabilize);
		setState(ControlState.PathFollowing);
	}
	
	public void followPath(Trajectory trajectory){
		followPath(trajectory, pidgey.getAngle());
	}
	
	public boolean isFinishedWithPath(){
		return isFinishedWithPath(1.0);
	}
	public boolean isFinishedWithPath(double percentCompleted){
		return (getState() == ControlState.PathFollowing && 
				brFollower.getSegment().position >= (percentCompleted * currentTrajectory.get(currentTrajectory.length()-1).position));
	}
	public boolean strictFinishedWithPath(){
		return (getState() == ControlState.PathFollowing &&
				flFollower.isFinished() && frFollower.isFinished() &&
				blFollower.isFinished() && brFollower.isFinished());
	}
	
	public void moveDistance(double wheelAngle, double distance){
		setState(ControlState.AdjustTargetDistance);
		for(SwerveDriveModule m : modules){
			m.setModuleAngle(wheelAngle);
		}
		for(SwerveDriveModule m : modules){
			m.moveInches(distance);
		}
		desiredWheelHeading = wheelAngle;
		setTargetHeading(pidgey.getAngle());
		setHeadingController(HeadingController.Stabilize);
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
	
	public void rotateAboutModule(boolean clockwiseRotation){
		setState(ControlState.ModuleRotation);
		
		double currentAngle = Util.boundAngleNeg180to180Degrees(pidgey.getAngle());
		
		if(currentAngle >= -45 && currentAngle <=45){
			if(clockwiseRotation){
				centerOfRotationModule = rearLeft;
			}else{
				centerOfRotationModule = rearRight;
			}
		}else if(currentAngle > 45 && currentAngle < 135){
			if(clockwiseRotation){
				centerOfRotationModule = rearRight;
			}else{
				centerOfRotationModule = frontRight;
			}
		}else if(currentAngle >= 135 || currentAngle <= -135){
			if(clockwiseRotation){
				centerOfRotationModule = frontRight;
			}else{
				centerOfRotationModule = frontLeft;
			}
		}else if(currentAngle < -45 && currentAngle > -135){
			if(clockwiseRotation){
				centerOfRotationModule = frontLeft;
			}else{
				centerOfRotationModule = rearLeft;
			}
		}
		
		oppositeModule = modules.get((centerOfRotationModule.arrayIndex() + 2) % 4);
		double tangentAngle = Math.toDegrees(Math.atan(Constants.WHEELBASE_WIDTH/Constants.WHEELBASE_LENGTH))+90;
		
		if(centerOfRotationModule.arrayIndex() % 2 == 1){
			lengthwiseModule = modules.get((centerOfRotationModule.arrayIndex() + 1) % 4);
			widthwiseModule = modules.get(Util.boundWithinModuleIndexRange(centerOfRotationModule.arrayIndex() - 1));
			if(oppositeModule.arrayIndex() == 1){
				oppositeModule.setModuleAngle(-tangentAngle + 180);
				lengthwiseModule.setModuleAngle(90);
				widthwiseModule.setModuleAngle(0);
			}else{
				oppositeModule.setModuleAngle(-tangentAngle);
				lengthwiseModule.setModuleAngle(-90);
				widthwiseModule.setModuleAngle(180);
			}
		}else{
			widthwiseModule = modules.get((centerOfRotationModule.arrayIndex() + 1) % 4);
			lengthwiseModule = modules.get(Util.boundWithinModuleIndexRange(centerOfRotationModule.arrayIndex() - 1));
			if(oppositeModule.arrayIndex() == 0){
				oppositeModule.setModuleAngle(tangentAngle);
				lengthwiseModule.setModuleAngle(90);
				widthwiseModule.setModuleAngle(180);
			}else{
				oppositeModule.setModuleAngle(tangentAngle + 180);
				lengthwiseModule.setModuleAngle(-90);
				widthwiseModule.setModuleAngle(0);
			}
		}
		
		centerOfRotationModule.setFieldRelativeAngle(0);
		
		setHeadingController(HeadingController.Off);
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
	
	public void purePursuit(Path path){
		if(currentState != ControlState.PurePursuit){
            zeroSensors();
            mPathFollower = new PathFollower(path, false,
                    new PathFollower.Parameters(
                            new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
                                    Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
                            Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                            Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                            Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                            Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
                            Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
                            Constants.kPathStopSteeringDistance));
            setTargetHeading(0.0);
            setHeadingController(HeadingController.Stabilize);
            setState(ControlState.PurePursuit);
            currentPath = path;
		}
	}
	
	private final Loop swerveLoop = new Loop(){
		@Override
		public void onStart(){
			stop();
		}
		@Override
		public void onLoop(){
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
			update();
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
		headingPID.setSetpoint(targetHeadingAngle);
		double heading = pidgey.getAngle();
		switch(headingController){
			case Off:
				targetHeadingAngle = pidgey.getAngle();
				break;
			case TemporaryDisable:
				setTargetHeading(pidgey.getAngle());
				if(now - disabledTimestamp >= 0.2){
					setHeadingController(HeadingController.Stabilize);
				}
				break;
			case Stabilize:
				if(!stabilizationTargetSet && (Timer.getFPGATimestamp() - manualRotationStopTime >= 0.45)){
					targetHeadingAngle = pidgey.getAngle();
					stabilizationTargetSet = true;
				}
				if(stabilizationTargetSet){
					if(Math.abs(getHeadingError()) > 5){
						headingPID.setPID(0.006, 0.0, 0.0, 0.0);
						//headingPID.setPID(0.0, 0.0, 0.0, 0.0);
						rotationCorrection = headingPID.calculate(heading, dt);
					}else if(Math.abs(getHeadingError()) > 0.5){
						headingPID.setPID(0.006, 0.0, 0.0, 0.0);
						//headingPID.setPID(0.0, 0.0, 0.0, 0.0);
						rotationCorrection = headingPID.calculate(heading, dt);
					}
				}
				break;
			case Snap:
				headingPID.setPID(0.005, 0.0, 0.001, 0.0);
				rotationCorrection = headingPID.calculate(heading, dt);
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
			    	for(int i=0; i<modules.size(); i++){
			    		if(Util.shouldReverse(kinematics.wheelAngles[i], modules.get(i).getModuleAngle())){
			    			modules.get(i).setModuleAngle(kinematics.wheelAngles[i] + 180);
			    		}else{
			    			modules.get(i).setModuleAngle(kinematics.wheelAngles[i]);
			    		}
			    	}
			    }
			    for(int i=0; i<modules.size(); i++){
		    		if(Util.shouldReverse(kinematics.wheelAngles[i], modules.get(i).getModuleAngle())){
		    			modules.get(i).setDriveOpenLoop(-kinematics.wheelSpeeds[i]);
		    			//modules.get(i).setDriveVelocity(-6);
		    		}else{
		    			modules.get(i).setDriveOpenLoop(kinematics.wheelSpeeds[i]);
		    			//modules.get(i).setDriveVelocity(6);
		    		}
			    }
				break;
			case PathFollowing:
				int futureSegmentIndex = 0;
				if((currentSegment + 6) < currentTrajectory.length())
					futureSegmentIndex = currentSegment + 6;
				else
					futureSegmentIndex = currentTrajectory.length() - 1;
				Segment futureSegment = currentTrajectory.get(futureSegmentIndex);
				Translation2d futurePos = new Translation2d(futureSegment.x, futureSegment.y);
				Translation2d currentPos = new Translation2d(robotX/12.0, -robotY/12.0);
				Translation2d deltaPos = futurePos.translateBy(currentPos.inverse());
				Rotation2d angleToFuturePos = deltaPos.direction();
				
				
				double fro = frFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
			    double flo = flFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
			    double blo = blFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
			    double bro = brFollower.calculate(distanceTraveled/12 - distanceTraveledOffset/12);
			    double pathWheelAngle = (frFollower.getHeading() + flFollower.getHeading() + 
			    		blFollower.getHeading() + brFollower.getHeading())/4;
			    pathWheelAngle = angleToFuturePos.getRadians();
			    double pidgeyAngle = Math.toRadians(pidgey.getAngle());
			    double x = Math.sin(pathWheelAngle);
			    double y = Math.cos(pathWheelAngle);
			    double tmp = (y* Math.cos(pidgeyAngle)) + (x * Math.sin(pidgeyAngle));
				xInput = (-y * Math.sin(pidgeyAngle)) + (x * Math.cos(pidgeyAngle));
				yInput = tmp;	
			    kinematics.calculate(xInput, yInput, rotationCorrection);
			    
			    setKinematicsAngles();
				frontRight.setDriveOpenLoop(fro);
				frontLeft.setDriveOpenLoop(flo);
				rearRight.setDriveOpenLoop(bro);
				rearLeft.setDriveOpenLoop(blo);
				
				if(brFollower.isFinished()){
					setState(ControlState.Neutral);
				}
				
				currentSegment++;
				break;
			case PurePursuit:
				double averageWheelHeading = -(frontRight.getModuleAngle() + frontLeft.getModuleAngle() + 
						rearLeft.getModuleAngle() + rearRight.getModuleAngle())/4;
				RigidTransform2d robot_pose = new RigidTransform2d(new Translation2d(robotX, robotY), Rotation2d.fromDegrees(averageWheelHeading));
		        Twist2d command = mPathFollower.update(Timer.getFPGATimestamp(), robot_pose,
		                distanceTraveled, rearRight.getModuleInchesPerSecond());
		        if (!mPathFollower.isFinished()) {
		            double goalAngle = -command.dtheta;
		            pidgeyAngle = Math.toRadians(pidgey.getAngle());
				    x = Math.sin(goalAngle);
				    y = Math.cos(goalAngle);
				    tmp = (y* Math.cos(pidgeyAngle)) + (x * Math.sin(pidgeyAngle));
					xInput = (-y * Math.sin(pidgeyAngle)) + (x * Math.cos(pidgeyAngle));
					yInput = tmp;	
				    kinematics.calculate(xInput, yInput, rotationCorrection);
				    System.out.println(-Math.toDegrees(command.dtheta));
				    setKinematicsAngles();
				    for(SwerveDriveModule m : modules){
				    	//m.setDriveVelocity(command.dx);
				    	m.setDriveVoltage(12 * (command.dx/156));
				    	//m.setFieldRelativeAngle(-command.dtheta);
				    }
		        } else {
		            setState(ControlState.Neutral);
		        }
				break;
			case AdjustTargetDistance:
			    pidgeyAngle = Math.toRadians(pidgey.getAngle());
			    xInput = Math.sin(Math.toRadians(desiredWheelHeading));
			    yInput = Math.cos(Math.toRadians(desiredWheelHeading));
			    kinematics.calculate(xInput, yInput, rotationCorrection);
			    setKinematicsAngles();
				break;
			case TurnInPlace:
				if(distanceOnTarget()){
					turnInPlace(targetHeadingAngle);
				}
				break;
			case BaseLock:
	
				break;
			case ModuleRotation:
				oppositeModule.setDriveOpenLoop(rotateInput);
				lengthwiseModule.setDriveOpenLoop(rotateInput * (Constants.WHEELBASE_LENGTH/Constants.SWERVE_R));
				widthwiseModule.setDriveOpenLoop(rotateInput * (Constants.WHEELBASE_WIDTH/Constants.SWERVE_R));
				centerOfRotationModule.setDriveOpenLoop(0);
				centerOfRotationModule.setFieldRelativeAngle(0);
				break;
			case Tank:
				for(SwerveDriveModule m : modules){
					m.setModuleAngle(0);
				}
				break;
			case Neutral:
				stop();
				break;
		}
		dt = now - timestamp;
		timestamp = now;
		if(getState() == ControlState.PathFollowing){
			System.out.println(dt);
		}
	}
	
	public void setTankOpenLoop(DriveSignal signal){
		frontRight.setDriveOpenLoop(signal.rightMotor);
		rearRight.setDriveOpenLoop(signal.rightMotor);
		frontLeft.setDriveOpenLoop(signal.leftMotor);
		rearLeft.setDriveOpenLoop(signal.leftMotor);
	}
	public void setModuleAngles(double angle){
		for(SwerveDriveModule m : modules){
			m.setModuleAngle(angle);
		}
	}
	public void setKinematicsAngles(){
		frontRight.setModuleAngle(kinematics.frSteeringAngle());
	    frontLeft.setModuleAngle(kinematics.flSteeringAngle());
	    rearLeft.setModuleAngle(kinematics.rlSteeringAngle());
	    rearRight.setModuleAngle(kinematics.rrSteeringAngle());
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
		zeroSensors(0);
	}
	public synchronized void zeroSensors(double heading){
		for(SwerveDriveModule m : modules){
			m.zeroSensors(heading);
		}
		robotX = 0;
		robotY = 0;
		distanceTraveled = 0;
		distanceTraveledOffset = 0;
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
