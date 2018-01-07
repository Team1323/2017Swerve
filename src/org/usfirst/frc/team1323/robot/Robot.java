package org.usfirst.frc.team1323.robot;

import java.util.Optional;

import com.team1323.frc2017.paths.PathContainer;
import com.team1323.frc2017.paths.StartToBoilerGearBlue;
import com.team254.lib.util.control.Path;

import Auto.AutoModeExecuter;
import Auto.SmartDashboardInteractions;
import IO.LogitechJoystick;
import IO.SimpleFlightStick;
import IO.SteeringWheel;
import IO.Xbox;
import Loops.LimelightProcessor;
import Loops.Looper;
import Loops.RobotStateEstimator;
import Loops.VisionProcessor;
import Subsystems.GearIntake;
import Subsystems.Hanger;
import Subsystems.RoboSystem;
import Subsystems.RobotState;
import Subsystems.Swerve;
import Subsystems.Turret;
import Utilities.CheesyDriveHelper;
import Utilities.Constants;
import Utilities.CrashTracker;
import Utilities.Logger;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.ShooterAimingParameters;
import Utilities.Util;
import Vision.VisionServer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RoboSystem robot;
	RobotState robotState;
	public Xbox driver, coDriver;
	public SimpleFlightStick leftDriver, rightDriver;
	public LogitechJoystick driverJoystick;
	public SteeringWheel steeringWheel;
	Looper enabledLooper = new Looper();
	Looper swerveLooper = new Looper();
	Looper disabledLooper = new Looper();
	AutoModeExecuter autoModeExecuter = null;
	private boolean sweeperNeedsToStop = false;
	private boolean sweeperUnjamComplete = false;
	private int cyclesReadyForShooting = 0;
	int cycles = 0;
	CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
	//VisionServer visionServer = VisionServer.getInstance();
	
	SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();
	
	PathContainer container = new StartToBoilerGearBlue();
	Path path;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try{
			Logger.clearLog();
			robot = RoboSystem.getInstance();
			robotState = RobotState.getInstance();
			CrashTracker.logRobotInit();
			//visionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());
			driver = new Xbox(0);
	        coDriver = new Xbox(1);
	        steeringWheel = new SteeringWheel(2);
	        driverJoystick = new LogitechJoystick(3);
	        zeroAllSensors();
	        robot.turret.resetAngle(90);
	        //enabledLooper.register(VisionProcessor.getInstance());
	        enabledLooper.register(LimelightProcessor.getInstance());
            enabledLooper.register(RobotStateEstimator.getInstance());
	        enabledLooper.register(robot.pidgey.getLoop());
	        enabledLooper.register(robot.turret.getLoop());
	        enabledLooper.register(robot.hanger.getLoop());
	        enabledLooper.register(robot.gearIntake.getLoop());
	        enabledLooper.register(robot.shooter.getLoop());
	        swerveLooper.register(robot.swerve.getLoop());
	        disabledLooper.register(robot.pidgey.getLoop());
	        //disabledLooper.register(VisionProcessor.getInstance());
	        disabledLooper.register(LimelightProcessor.getInstance());
	        disabledLooper.register(RobotStateEstimator.getInstance());
	        robot.initCamera();
	        smartDashboardInteractions.initWithDefaults();
	        
	        System.out.println(smartDashboardInteractions.getSelectedSide());
	        
	        if(smartDashboardInteractions.getSelectedMode().equals("Hopper")){
	        	if(smartDashboardInteractions.getSelectedSide().equals("Blue")){
	        		robot.pidgey.setAngle(180);
	        		robot.turret.resetAngle(90);
	        		robot.swerve.zeroSensors(180);
	        	}else{
	        		robot.pidgey.setAngle(0);
	        		robot.turret.resetAngle(-90);
	        		robot.swerve.zeroSensors(0);
	        	}
	        }else if(smartDashboardInteractions.getSelectedMode().equals("Gear and Hopper")){
	        	if(smartDashboardInteractions.getSelectedSide().equals("Blue")){
		        	robot.pidgey.setAngle(0);
		        	robot.turret.resetAngle(90);
		        	robot.swerve.zeroSensors(0);
	        	}else{
	        		robot.pidgey.setAngle(0);
		        	robot.turret.resetAngle(-90);
		        	robot.swerve.zeroSensors(0);
	        	}
	        }else if(smartDashboardInteractions.getSelectedMode().equals("Middle Gear")){
	        	robot.pidgey.setAngle(0);
        		robot.turret.resetAngle(-90);
        		robot.swerve.zeroSensors(0);
	        }
	        
	        path = container.buildPath();
	        
	        VisionServer.getInstance();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	public void zeroAllSensors(){
		robot.swerve.zeroSensors();
		//robot.pidgey.setAngle(0);
		robotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d(), new Rotation2d());
	}
	public void outputAllToSmartDashboard(){
		robot.swerve.outputToSmartDashboard();
		robot.intake.outputToSmartDashboard();
		robot.pidgey.outputToSmartDashboard();
		robot.turret.outputToSmartDashboard();
		robot.hanger.outputToSmartDashboard();
		robot.gearIntake.outputToSmartDashboard();
		robot.shooter.outputToSmartDashboard();
		robot.sweeper.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		//visionServer.outputToSmartDashboard();
	}
	public void stopAll(){
		robot.swerve.stop();
		robot.intake.stop();
		robot.turret.stop();
		robot.hanger.stop();
		robot.gearIntake.stop();
		robot.shooter.stop();
		robot.sweeper.stop();
	}
	public void coDriverStop(){
		robot.intake.stop();
		robot.hanger.stop();
		robot.gearIntake.coDriverStop();
		robot.shooter.stop();
		robot.sweeper.stop();
	}
	@Override
	public void disabledInit(){
		try{
			CrashTracker.logDisabledInit();
			
			if(autoModeExecuter != null){
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;
			
			enabledLooper.stop();
			swerveLooper.stop();
			disabledLooper.start();
			stopAll();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	@Override
	public void autonomousInit() {
		try{
			CrashTracker.logAutoInit();
			if(autoModeExecuter != null){
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;
			
			zeroAllSensors();
			
			disabledLooper.stop();
			enabledLooper.start();
			swerveLooper.start();
			
			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
			autoModeExecuter.start();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	@Override
	public void teleopInit(){
		try{
			CrashTracker.logTeleopInit();
			
			if(autoModeExecuter != null){
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;
			
			disabledLooper.stop();
			enabledLooper.start();
			swerveLooper.start();
			
			robot.swerve.setState(Swerve.ControlState.Neutral);
			robot.swerve.setTargetHeading(robot.pidgey.getAngle());
			//robot.gearIntake.setState(GearIntake.State.RETRACTED_HOLDING);
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	@Override
	public void disabledPeriodic(){
		try{
			stopAll();
			outputAllToSmartDashboard();
			smartDashboardInteractions.output();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		try{
			outputAllToSmartDashboard();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		try{
			driver.update();
			coDriver.update();
			//driverJoystick.update();
			//steeringWheel.update();
			
			driverXboxControls();
			coDriverXboxControls();
			
			outputAllToSmartDashboard();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	public void driverXboxControls(){
		//Swerve
		if(robot.swerve.getState() == Swerve.ControlState.Tank){
			robot.swerve.setTankOpenLoop(cheesyDriveHelper.cheesyDrive(-driverJoystick.getY(), steeringWheel.getWheelTurn(), steeringWheel.leftBumper.isBeingPressed(), true));
		}else{
			robot.swerve.sendInput(driver.getX(Hand.kLeft), -driver.getY(Hand.kLeft), driver.getX(Hand.kRight), false, driver.getTriggerAxis(Hand.kLeft) > 0);
		}
		if(driver.leftCenterClick.wasPressed()){
			robot.swerve.toggleSuperSlow();
		}
		if(driver.getBackButton()){
			robot.pidgey.setAngle(0);
			robot.swerve.setTargetHeading(0.0);
			robot.swerve.temporarilyDisable();
		}
		if(driver.getAButton()){
			robot.swerve.rotate(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 180));
		}else if(driver.getBButton()){
			robot.swerve.rotate(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 90));
		}else if(driver.getXButton()){
			//robot.swerve.rotate(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 270));
			if(robot.turret.isStationary()){
				Optional<ShooterAimingParameters> params = robotState.getAimingParameters(Timer.getFPGATimestamp());
				if(params.isPresent()){
					robot.swerve.moveDistance(Util.boundAngle0to360Degrees(robot.turret.turretAngleToWheelAngle(-params.get().getTurretAngle().getDegrees())), params.get().getRange() - Constants.kOptimalShootingDistance);
				}
			}
			robot.turret.enableVision();
		}else if(driver.getYButton()){
			robot.swerve.rotate(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 0));
		}else if(driver.rightBumper.wasPressed()){
			robot.swerve.rotate(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 60));
		}else if(driver.leftBumper.wasPressed()){
			robot.swerve.rotate(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), -60));
		}
		
		if(robot.hanger.isHanging()){
			robot.swerve.setHeadingController(Swerve.HeadingController.Off);
		}
		
		if(driver.getPOV() == 90){
			robot.swerve.rotateAboutModule(false);
		}else if(driver.getPOV() == 180){
			//robot.swerve.followPath(robot.swerve.rightPegTrajectory, Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), -60));
		}else if(driver.getPOV() == 270){
			robot.swerve.rotateAboutModule(true);
		}else if(driver.getPOV() == 0){
			//robot.swerve.purePursuit(path);
		}
		
		if(driver.POV180.wasPressed()){
			NetworkTable table = NetworkTable.getTable("limelight");
			if(table.getNumber("ledMode", 0) == 0){
				table.putNumber("ledMode", 1);
				table.putNumber("camMode", 1);
			}else{
				table.putNumber("ledMode", 0);
				table.putNumber("camMode", 0);
			}
		}
		
		//Gear Score
		if(driver.getTriggerAxis(Hand.kRight) > 0){
			robot.gearIntake.score();
		}
		
		//Hanger
		if(driver.getStartButton()){
			robot.hanger.startHang();
			robot.swerve.setLowPower(true);
			robot.retractBallFlap();
		}
	}
	
	public void coDriverXboxControls(){
		
		if(coDriver.getBackButton()){
			sweeperUnjamComplete = false;
			sweeperNeedsToStop = false;
			coDriverStop();
			robot.swerve.setLowPower(false);
			robot.extendBallFlap();
			robot.turret.lock();
		}
		
		//Ball Intake
		if(coDriver.getBumper(Hand.kRight)){
    		robot.intake.intakeForward();
    	}else if(coDriver.getBumper(Hand.kLeft)){
    		robot.intake.intakeReverse();
    	}
		
		//Vision
		
		
		//Turret
		if(Math.abs(coDriver.getX(Hand.kRight)) > 0){
			robot.turret.setState(Turret.ControlState.Manual);
			robot.turret.setPercentVBus(coDriver.getX(Hand.kRight)*0.6);
		}else if(Math.abs(coDriver.getX(Hand.kLeft)) > 0){
			robot.turret.setState(Turret.ControlState.Manual);
			robot.turret.setPercentVBus(coDriver.getX(Hand.kLeft)*0.15);
		}else if(coDriver.getStickButton(Hand.kRight)){
			robot.turret.setMotionMagic(90);
		}else if(coDriver.getPOV() == 90){
			robot.turret.setMotionMagic(110);
		}else if(coDriver.getPOV() == 270){
			robot.turret.setMotionMagic(45);
		}else if(coDriver.xButton.wasPressed()){
			if(robot.turret.isStationary()){
				Optional<ShooterAimingParameters> params = robotState.getAimingParameters(Timer.getFPGATimestamp());
				if(params.isPresent()){
					robot.swerve.moveDistance(Util.boundAngle0to360Degrees(robot.turret.turretAngleToWheelAngle(-params.get().getTurretAngle().getDegrees())), params.get().getRange() - Constants.kOptimalShootingDistance);
				}
			}
			robot.turret.enableVision();
		}else if(robot.turret.getCurrentState() == Turret.ControlState.Manual){
			robot.turret.lock();
		}
		
		if(robot.turret.isTracking() && robot.turret.onTarget() && !robot.sweeper.isFeeding()){
			Optional<ShooterAimingParameters> params = robotState.getAimingParameters(Timer.getFPGATimestamp());
			if(params.isPresent()){
				if(Math.abs(params.get().getRange() - Constants.kOptimalShootingDistance) <= 1.0){
					coDriver.rumble(1, 1);
					driver.rumble(1,  1);
					cyclesReadyForShooting++;
				}else if(robot.swerve.distanceOnTarget()){
					robot.swerve.moveDistance(robot.turret.turretAngleToWheelAngle(-params.get().getTurretAngle().getDegrees()), params.get().getRange() - Constants.kOptimalShootingDistance);
				}
			}
		}else if(robot.sweeper.isFeeding()){
			cyclesReadyForShooting = 0;
		}
		
		if(coDriver.leftCenterClick.wasPressed()){
			//visionServer.requestAppRestart();
		}
		
		//Shooter
		if(coDriver.getTriggerAxis(Hand.kLeft) > 0 || cyclesReadyForShooting >= 3){
			robot.turret.gyroLock();
			robot.swerve.baseLock();
			robot.shooter.setSpinUp(Constants.SHOOTING_SPEED);
			sweeperNeedsToStop = false;
		}
		
		//Sweeper
		if(((coDriver.getTriggerAxis(Hand.kRight) > 0) || sweeperUnjamComplete || cyclesReadyForShooting >= 3) 
				&& robot.shooter.onTarget() && !robot.sweeper.isFeeding()){
			robot.sweeper.startSweeper();
			cyclesReadyForShooting = 0;
		}
		if(coDriver.getYButton()){
			sweeperUnjamComplete = false;
			robot.sweeper.stop();
			robot.sweeper.rollerReverse();
			sweeperNeedsToStop = true;
		}else if(sweeperNeedsToStop){
			robot.sweeper.stop();
			sweeperNeedsToStop = false;
			sweeperUnjamComplete = true;
		}
		
		//Ball Flap
		
		
		//Gear Intake
		if(coDriver.getAButton()){
			robot.gearIntake.extend();
		}else if(coDriver.getBButton()){
			if(robot.gearIntake.getState() == GearIntake.State.RETRACTED_HOLDING){
				robot.gearIntake.setState(GearIntake.State.RETRACTED_SECURING);
			}else{
				robot.gearIntake.retract();
			}
		}else if(coDriver.getPOV() == 0){
			robot.gearIntake.setState(GearIntake.State.REVERSED);
		}else if(robot.gearIntake.getState() == GearIntake.State.RETRACTED_SECURING){
			robot.gearIntake.setState(GearIntake.State.RETRACTED_HOLDING);
		}
		
		if(robot.gearIntake.needsToNotifyGearAcquired()){
			coDriver.rumble(3, 2);
			driver.rumble(3, 2);
		}
		if(robot.gearIntake.needsToNotifyGearLoss()){
			coDriver.rumble(1, 2);
			driver.rumble(1, 2);
		}
		
		//Hanger
		if(coDriver.startButton.wasPressed()){
			robot.hanger.startHang();
			robot.swerve.setLowPower(true);
			robot.retractBallFlap();
		}
		if(coDriver.getPOV() == 180){
			robot.hanger.setState(Hanger.State.WEAK_HANG);
		}
	}
	public void oneControllerMode(){
		//Swerve
		robot.swerve.sendInput(driver.getX(Hand.kLeft), -driver.getY(Hand.kLeft), driver.getX(Hand.kRight), false, driver.getTriggerAxis(Hand.kLeft) > 0);
		if(driver.backButton.wasPressed()){
			coDriverStop();
			robot.swerve.setLowPower(false);
			robot.extendBallFlap();
		}else if(driver.backButton.longPressed()){
			robot.pidgey.setAngle(0);
			robot.swerve.setTargetHeading(0.0);
		}
		if(driver.aButton.longPressed()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 180));
		}else if(driver.bButton.longPressed()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 90));
		}else if(driver.xButton.longPressed()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 270));
		}else if(driver.yButton.longPressed()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 0));
		}
		//Ball Intake
		if(driver.rightBumper.wasPressed()){
    		robot.intake.toggleForward();
    	}else if(driver.leftBumper.wasPressed()){
    		robot.intake.toggleReverse();
    	}
		//Turret
		if(Math.abs(driver.getY(Hand.kRight)) > 0){
			robot.turret.setState(Turret.ControlState.Manual);
			robot.turret.setPercentVBus(driver.getY(Hand.kRight)*0.3);
		}else if(driver.leftCenterClick.wasPressed() || driver.rightCenterClick.wasPressed()){
			robot.turret.setMotionMagic(90);
		}else if(driver.xButton.wasPressed()){
			robot.turret.enableVision();
		}else if(driver.getPOV() == 180){
			robot.turret.setMotionMagic(-90);
		}else if(robot.turret.getCurrentState() == Turret.ControlState.Manual){
			robot.turret.lock();
		}
		
		if(robot.turret.getCurrentState() == Turret.ControlState.VisionTracking && robotState.getTargetVisbility() && robot.turret.isStationary() && robotState.getVisionAngle() < 1.5){
			driver.rumble(1, 1);
		}
		
		if(driver.startButton.longPressed()){
			//visionServer.requestAppRestart();
		}
		//Shooter
		if(driver.rightTrigger.longPressed()){
			robot.turret.fieldPositionLock();
			robot.shooter.setSpeed(Constants.SHOOTING_SPEED);
			sweeperUnjamComplete = true;
		}
		//Sweeper
		if(sweeperUnjamComplete && robot.shooter.onTarget()){
			robot.sweeper.startSweeper();
			sweeperUnjamComplete = false;
		}
		if(driver.yButton.wasPressed()){
			if(sweeperNeedsToStop){
				robot.sweeper.stop();
				sweeperNeedsToStop = false;
				sweeperUnjamComplete =  true;
			}else{
				robot.sweeper.stop();
				robot.sweeper.rollerReverse();
				sweeperNeedsToStop = true;
			}
		}
		//Gear Intake
		if(driver.aButton.wasPressed()){
			robot.gearIntake.extend();
		}else if(driver.bButton.wasPressed()){
			robot.gearIntake.retract();
		}else if(driver.rightTrigger.wasPressed()){
			robot.gearIntake.score();
		}
		if(robot.gearIntake.needsToNotifyGearAcquired()){
			driver.rumble(3, 2);
		}
		if(robot.gearIntake.needsToNotifyGearLoss()){
			driver.rumble(1, 2);
		}
		//Hanger
		if(driver.startButton.isBeingPressed()){
			robot.hanger.startHang();
			robot.swerve.setLowPower(true);
			robot.retractBallFlap();
		}
	}
}

