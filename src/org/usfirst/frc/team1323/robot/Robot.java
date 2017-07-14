package org.usfirst.frc.team1323.robot;

import IO.FlightStick;
import IO.SimpleXbox;
import Loops.Looper;
import Subsystems.GearIntake;
import Subsystems.RoboSystem;
import Subsystems.Swerve.HeadingController;
import Subsystems.Turret;
import Utilities.Constants;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;

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
	public FlightStick leftDriver, rightDriver;
	Looper enabledLooper = new Looper();
	Looper disabledLooper = new Looper();
	private boolean sweeperNeedsToStop = false;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		driver = new SimpleXbox(0);
        //driver.start();
        coDriver = new SimpleXbox(1);
        //coDriver.start();
        /*leftDriver = new FlightStick(2);
        rightDriver = new FlightStick(3);
        leftDriver.start();
        rightDriver.start();*/
        zeroAllSensors();
        robot.turret.resetAngle(90);
        enabledLooper.register(robot.swerve.getLoop());
        enabledLooper.register(robot.intake.getPidgeonLoop());
        enabledLooper.register(robot.turret.getLoop());
        enabledLooper.register(robot.hanger.getLoop());
        enabledLooper.register(robot.gearIntake.getLoop());
        disabledLooper.register(robot.intake.getPidgeonLoop());
	}
	public void zeroAllSensors(){
		robot.swerve.zeroSensors();
		robot.intake.zeroSensors();
		robot.turret.zeroSensors();
	}
	public void outputAllToSmartDashboard(){
		robot.swerve.outputToSmartDashboard();
		robot.intake.outputToSmartDashboard();
		robot.turret.outputToSmartDashboard();
		robot.hanger.outputToSmartDashboard();
		robot.gearIntake.outputToSmartDashboard();
		robot.shooter.outputToSmartDashboard();
		robot.sweeper.outputToSmartDashboard();
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
		enabledLooper.stop();
		disabledLooper.start();
		stopAll();
	}
	@Override
	public void autonomousInit() {
		
	}
	@Override
	public void teleopInit(){
		disabledLooper.stop();
		enabledLooper.start();
		robot.swerve.setTargetHeading(robot.intake.pidgey.getAngle());
	}
	@Override
	public void disabledPeriodic(){
		stopAll();
		outputAllToSmartDashboard();
	}
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//Driver
		//Swerve
		robot.swerve.sendInput(driver.getX(Hand.kLeft), -driver.getY(Hand.kLeft), driver.getX(Hand.kRight), false, driver.getTriggerAxis(Hand.kLeft) > 0);
		//robot.swerve.sendInput(leftDriver.getXAxis(), -leftDriver.getYAxis(), rightDriver.getXAxis(), false);
		if(driver.getBackButton()){
			robot.intake.pidgey.setAngle(0);
			robot.swerve.setTargetHeading(0.0);
		}
		if(driver.getAButton()){
			robot.swerve.setSnapAngle(180);
		}else if(driver.getBButton()){
			robot.swerve.setSnapAngle(90);
		}else if(driver.getXButton()){
			robot.swerve.setSnapAngle(-90);
		}else if(driver.getYButton()){
			robot.swerve.setSnapAngle(0);
		}else{
			robot.swerve.setHeadingController(HeadingController.Stabilize);
		}
		//Gear Score
		if(driver.getTriggerAxis(Hand.kRight) > 0){
			robot.gearIntake.score();
		}
		
		
		
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
			robot.sweeper.rollerReverse();
			sweeperNeedsToStop = true;
		}else if(sweeperNeedsToStop){
			robot.sweeper.stop();
			sweeperNeedsToStop = false;
		}
		//Gear Intake
		if(coDriver.getAButton()){
			robot.gearIntake.intakeGear();
		}else if(coDriver.getBButton()){
			robot.gearIntake.retract();
		}else if(coDriver.getPOV() == 0){
			robot.gearIntake.setState(GearIntake.State.REVERSED);
		}
		
		//Hanger
		if(coDriver.getStartButton() || driver.getStartButton()){
			robot.hanger.startHang();
			robot.swerve.setLowPower(true);
		}
		
		if(coDriver.getBackButton()){
			coDriverStop();
			robot.swerve.setLowPower(false);
		}
		
		outputAllToSmartDashboard();
	}
}

