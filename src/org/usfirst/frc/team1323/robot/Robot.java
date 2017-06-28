package org.usfirst.frc.team1323.robot;

import IO.Controller;
import Loops.Looper;
import Subsystems.RoboSystem;
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
	public Controller driver, coDriver;
	Looper enabledLooper = new Looper();
	Looper disabledLooper = new Looper();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		driver = new Controller(0);
        driver.start();
        coDriver = new Controller(1);
        coDriver.start();
        zeroAllSensors();
        enabledLooper.register(robot.swerve.getLoop());
        enabledLooper.register(robot.intake.getPidgeonLoop());
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
	}
	public void stopAll(){
		robot.swerve.stop();
		robot.intake.stop();
		robot.turret.stop();
	}
	public void coDriverStop(){
		robot.intake.stop();
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
		robot.swerve.sendInput(driver.getX(Hand.kLeft), driver.getY(Hand.kLeft), driver.getX(Hand.kRight), false);
		if(coDriver.rightBumper.isPressed()){
    		robot.intake.intakeForward();
    	}else if(coDriver.leftBumper.isPressed()){
    		robot.intake.intakeReverse();
    	}
		if(coDriver.backButton.isPressed()){
			coDriverStop();
		}
		
		outputAllToSmartDashboard();
	}
}

