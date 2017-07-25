package org.usfirst.frc.team1323.robot;

import IO.SimpleFlightStick;
import IO.SimpleXbox;
import Loops.Looper;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;
import Subsystems.Turret;
import Utilities.Constants;
import Utilities.CrashTracker;
import Utilities.Util;
import Vision.VisionServer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RoboSystem robot = RoboSystem.getInstance();
	public SimpleXbox driver, coDriver;
	public SimpleFlightStick leftDriver, rightDriver;
	Looper enabledLooper = new Looper();
	Looper disabledLooper = new Looper();
	private boolean sweeperNeedsToStop = false;
	VisionServer visionServer = VisionServer.getInstance();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try{
			CrashTracker.logRobotInit();
			driver = new SimpleXbox(0);
	        coDriver = new SimpleXbox(1);
	        leftDriver = new SimpleFlightStick(2);
	        rightDriver = new SimpleFlightStick(3);
	        zeroAllSensors();
	        robot.turret.resetAngle(90);
	        enabledLooper.register(robot.swerve.getLoop());
	        enabledLooper.register(robot.pidgey.getLoop());
	        enabledLooper.register(robot.turret.getLoop());
	        enabledLooper.register(robot.hanger.getLoop());
	        enabledLooper.register(robot.gearIntake.getLoop());
	        disabledLooper.register(robot.pidgey.getLoop());
	        
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
		
		SmartDashboard.putNumber("Left Joystick X", leftDriver.getRawAxis(0));
		SmartDashboard.putNumber("Left Joystick Y", leftDriver.getRawAxis(1));
		SmartDashboard.putNumber("Right Joystick X", rightDriver.getRawAxis(0));
		SmartDashboard.putNumber("Right Joystick Y", rightDriver.getRawAxis(1));
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
			//Driver
			//driverXboxControls();
			driverFlightStickControls();
			
			//Co Driver
			
			//Ball Intake
			if(coDriver.getBumper(Hand.kRight)){
	    		robot.intake.intakeForward();
	    	}else if(coDriver.getBumper(Hand.kLeft)){
	    		robot.intake.intakeReverse();
	    	}
			//Turret
			if(Math.abs(coDriver.getX(Hand.kRight)) > 0){
				robot.turret.setState(Turret.ControlState.Manual);
				robot.turret.setPercentVBus(coDriver.getX(Hand.kRight)*0.3);
			}else if(coDriver.getStickButton(Hand.kLeft) || coDriver.getStickButton(Hand.kRight)){
				robot.turret.setAngle(90);
			}else if(coDriver.getPOV() == 180){
				robot.turret.setAngle(-90);
			}else if(robot.turret.getCurrentState() == Turret.ControlState.Manual){
				robot.turret.lock();
			}
			//Shooter
			if(coDriver.getTriggerAxis(Hand.kLeft) > 0){
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
			
			/*if(robot.gearIntake.isExtended() && robot.gearIntake.hasGear()){
				coDriver.rumble(3, 1);
				driver.rumble(3, 1);
			}*/
			if(robot.gearIntake.needsToNotifyGearAcquired()){
				coDriver.rumble(3, 2);
				driver.rumble(3, 2);
			}
			if(robot.gearIntake.needsToNotifyGearLoss()){
				coDriver.rumble(1, 2);
				driver.rumble(1, 2);
			}
			
			if(coDriver.getBackButton()){
				coDriverStop();
				robot.swerve.setLowPower(false);
				robot.extendBallFlap();
			}
			
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
		}
		//Gear Score
		if(driver.getTriggerAxis(Hand.kRight) > 0){
			robot.gearIntake.score();
		}
		//Hanger
		if(coDriver.getStartButton() || driver.getStartButton()){
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
		//Hanger
		if(coDriver.getStartButton()){
			robot.hanger.startHang();
			robot.swerve.setLowPower(true);
			robot.retractBallFlap();
		}
	}
}

