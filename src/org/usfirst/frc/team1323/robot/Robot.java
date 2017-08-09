package org.usfirst.frc.team1323.robot;

import IO.LogitechJoystick;
import IO.SimpleFlightStick;
import IO.SteeringWheel;
import IO.Xbox;
import Loops.Looper;
import Loops.RobotStateEstimator;
import Loops.VisionProcessor;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;
import Subsystems.RobotState;
import Subsystems.Turret;
import Utilities.Constants;
import Utilities.CrashTracker;
import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.Util;
import Vision.VisionServer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RoboSystem robot = RoboSystem.getInstance();
	RobotState robotState = RobotState.getInstance();
	public Xbox driver, coDriver;
	public SimpleFlightStick leftDriver, rightDriver;
	public LogitechJoystick driverJoystick;
	public SteeringWheel steeringWheel;
	Looper enabledLooper = new Looper();
	Looper disabledLooper = new Looper();
	private boolean sweeperNeedsToStop = false;
	private boolean sweeperCanTurnOn = false;
	VisionServer visionServer = VisionServer.getInstance();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try{
			CrashTracker.logRobotInit();
			visionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());
			driver = new Xbox(0);
	        coDriver = new Xbox(1);
	        /*leftDriver = new SimpleFlightStick(2);
	        rightDriver = new SimpleFlightStick(3);*/
	        driverJoystick = new LogitechJoystick(3);
	        steeringWheel = new SteeringWheel(2);
	        zeroAllSensors();
	        robot.turret.resetAngle(90);
	        enabledLooper.register(VisionProcessor.getInstance());
            enabledLooper.register(RobotStateEstimator.getInstance());
	        enabledLooper.register(robot.swerve.getLoop());
	        enabledLooper.register(robot.pidgey.getLoop());
	        enabledLooper.register(robot.turret.getLoop());
	        enabledLooper.register(robot.hanger.getLoop());
	        enabledLooper.register(robot.gearIntake.getLoop());
	        disabledLooper.register(robot.pidgey.getLoop());
	        disabledLooper.register(VisionProcessor.getInstance());
	        disabledLooper.register(RobotStateEstimator.getInstance());
	        
	        VisionServer.getInstance();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	public void zeroAllSensors(){
		robot.swerve.zeroSensors();
		robot.pidgey.setAngle(0);
		robot.turret.zeroSensors();
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
		visionServer.outputToSmartDashboard();
		
		/*SmartDashboard.putNumber("Left Joystick X", leftDriver.getRawAxis(0));
		SmartDashboard.putNumber("Left Joystick Y", leftDriver.getRawAxis(1));
		SmartDashboard.putNumber("Right Joystick X", rightDriver.getRawAxis(0));
		SmartDashboard.putNumber("Right Joystick Y", rightDriver.getRawAxis(1));*/
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
			enabledLooper.stop();
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
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	@Override
	public void teleopInit(){
		try{
			CrashTracker.logTeleopInit();
			disabledLooper.stop();
			enabledLooper.start();
			robot.swerve.setTargetHeading(robot.pidgey.getAngle());
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
			/*driverJoystick.update();
			steeringWheel.update();*/
			
			driverXboxControls();
			//driverFlightStickControls();
			coDriverXboxControls();
			//oneControllerMode();
			
			/*robot.swerve.sendInput(driverJoystick.getXAxis(), -driverJoystick.getYAxis(), steeringWheel.getWheelTurn(), false, steeringWheel.leftBumper.isBeingPressed());
			switch(driverJoystick.getPOV()){
				case 0:
					robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 0));
					break;
				case 90:
					robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 90));
					break;
				case 180:
					robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 180));
					break;
				case 270:
					robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 270));
					break;
			}
			if(driverJoystick.topLeft.wasPressed()){
				robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), -60));
			}else if(driverJoystick.topRight.wasPressed()){
				robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 60));
			}
			if(steeringWheel.yButton.wasPressed()){
				robot.pidgey.setAngle(0);
				robot.swerve.setTargetHeading(0);
			}
			if(driverJoystick.triggerButton.wasPressed()){
				robot.gearIntake.score();
			}*/
			
			outputAllToSmartDashboard();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	public void driverXboxControls(){
		//Swerve
		robot.swerve.sendInput(driver.getX(Hand.kLeft), -driver.getY(Hand.kLeft), driver.getX(Hand.kRight), false, driver.getTriggerAxis(Hand.kLeft) > 0);
		if(driver.getBackButton()){
			robot.pidgey.setAngle(0);
			robot.swerve.setTargetHeading(0.0);
		}
		if(driver.getAButton()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 180));
		}else if(driver.getBButton()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 90));
		}else if(driver.getXButton()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 270));
		}else if(driver.getYButton()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 0));
		}else if(driver.rightBumper.wasPressed()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 60));
		}else if(driver.leftBumper.wasPressed()){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), -60));
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
	public void driverFlightStickControls(){
		//Swerve
		robot.swerve.sendInput(leftDriver.getXAxis(), -leftDriver.getYAxis(), rightDriver.getX(), false, leftDriver.getTriggerButton());
		if(leftDriver.getDownButton()){
			robot.pidgey.setAngle(0);
			robot.swerve.setTargetHeading(0.0);
		}
		if(rightDriver.getDownButton() || rightDriver.getPOV() == 180){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 180));
		}else if(rightDriver.getLeftButton() || rightDriver.getPOV() == 90){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 90));
		}else if(rightDriver.getRightButton() || rightDriver.getPOV() == 270){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 270));
		}else if(rightDriver.getPOV() == 0){
			robot.swerve.setSnapAngle(Util.placeInAppropriate0To360Scope(robot.pidgey.getAngle(), 0));
		}
		if(leftDriver.getPOV() >= 0){
			robot.pidgey.setAngle((leftDriver.getPOV()));
			robot.swerve.setTargetHeading(leftDriver.getPOV());
		}
		//Gear Score
		if(rightDriver.getTriggerButton()){
			robot.gearIntake.score();
		}
	}
	public void coDriverXboxControls(){
		//Ball Intake
		if(coDriver.getBumper(Hand.kRight)){
    		robot.intake.intakeForward();
    	}else if(coDriver.getBumper(Hand.kLeft)){
    		robot.intake.intakeReverse();
    	}
		//Turret
		if(Math.abs(coDriver.getX(Hand.kRight)) > 0){
			robot.turret.setState(Turret.ControlState.Manual);
			robot.turret.setPercentVBus(coDriver.getX(Hand.kRight)*0.5);
		}else if(Math.abs(coDriver.getX(Hand.kLeft)) > 0){
			robot.turret.setState(Turret.ControlState.Manual);
			robot.turret.setPercentVBus(coDriver.getX(Hand.kLeft)*0.25);
		}else if(coDriver.getStickButton(Hand.kLeft) || coDriver.getStickButton(Hand.kRight)){
			robot.turret.setSnapAngle(90);
		}else if(coDriver.getPOV() == 180){
			robot.turret.setSnapAngle(-90);
		}else if(coDriver.getPOV() == 90){
			robot.turret.setSnapAngle(110);
		}else if(coDriver.getPOV() == 270){
			robot.turret.setSnapAngle(45);
		}else if(coDriver.xButton.wasPressed()){
			//robot.turret.setState(Turret.ControlState.CalculatedTracking);
			System.out.println(Double.toString(robotState.getAimingParameters(Timer.getFPGATimestamp()).getTurretAngle().getDegrees()));
			robot.turret.setState(Turret.ControlState.VisionTracking);
			//robot.turret.moveDegrees(-robotState.getAimingParameters(Timer.getFPGATimestamp()).getTurretAngle().getDegrees());
		}else if(robot.turret.getCurrentState() == Turret.ControlState.Manual){
			robot.turret.lock();
		}
		
		if(robot.turret.getCurrentState() == Turret.ControlState.VisionTracking && robotState.getTargetVisbility() && robot.turret.onTarget()){
			if(!coDriver.isRumbling())coDriver.rumble(1, 1);
		}
		
		if(coDriver.startButton.longPressed()){
			visionServer.requestAppRestart();
		}
		//Shooter
		if(coDriver.getTriggerAxis(Hand.kLeft) > 0){
			robot.turret.gyroLock();
			robot.shooter.setSpeed(Constants.SHOOTING_SPEED);
		}
		//Sweeper
		if(coDriver.getTriggerAxis(Hand.kRight) > 0 && robot.shooter.onTarget()){
			robot.sweeper.startSweeper();
		}
		if(coDriver.getYButton()){
			robot.sweeper.stop();
			robot.sweeper.rollerReverse();
			sweeperNeedsToStop = true;
		}else if(sweeperNeedsToStop){
			robot.sweeper.stop();
			sweeperNeedsToStop = false;
		}
		//Gear Intake
		if(coDriver.getAButton()){
			robot.gearIntake.extend();
		}else if(coDriver.getBButton()){
			robot.gearIntake.retract();
		}else if(coDriver.getPOV() == 0){
			robot.gearIntake.setState(GearIntake.State.REVERSED);
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
		
		if(coDriver.getBackButton()){
			coDriverStop();
			robot.swerve.setLowPower(false);
			robot.extendBallFlap();
			robot.turret.lock();
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
			robot.turret.setSnapAngle(90);
		}else if(driver.xButton.wasPressed()){
			robot.turret.setState(Turret.ControlState.CalculatedTracking);
		}else if(driver.getPOV() == 180){
			robot.turret.setSnapAngle(-90);
		}else if(robot.turret.getCurrentState() == Turret.ControlState.Manual){
			robot.turret.lock();
		}
		//Shooter
		if(driver.rightTrigger.longPressed()){
			robot.shooter.setSpeed(Constants.SHOOTING_SPEED);
			sweeperCanTurnOn = true;
		}
		//Sweeper
		if(sweeperCanTurnOn && robot.shooter.onTarget()){
			robot.sweeper.startSweeper();
			sweeperCanTurnOn = false;
		}
		if(driver.yButton.wasPressed()){
			if(sweeperNeedsToStop){
				robot.sweeper.stop();
				sweeperNeedsToStop = false;
				sweeperCanTurnOn =  true;
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

