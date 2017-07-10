package org.usfirst.frc.team1323.robot;

import IO.FlightStick;
import IO.SimpleXbox;
import IO.Xbox;
import Loops.Looper;
import Subsystems.RoboSystem;
import Subsystems.Turret;
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
	}
	public void stopAll(){
		robot.swerve.stop();
		robot.intake.stop();
		robot.turret.stop();
		robot.hanger.stop();
		robot.gearIntake.stop();
	}
	public void coDriverStop(){
		robot.intake.stop();
		robot.hanger.stop();
		robot.gearIntake.coDriverStop();
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
		robot.swerve.sendInput(driver.getX(Hand.kLeft), -driver.getY(Hand.kLeft), driver.getX(Hand.kRight), false);
		//robot.swerve.sendInput(leftDriver.getXAxis(), -leftDriver.getYAxis(), rightDriver.getXAxis(), false);
		if(driver.getBackButton()){
			robot.intake.pidgey.setAngle(0);
		}
		if(coDriver.getBumper(Hand.kRight)){
    		robot.intake.intakeForward();
    	}else if(coDriver.getBumper(Hand.kLeft)){
    		robot.intake.intakeReverse();
    	}
		if(coDriver.getBackButton()){
			coDriverStop();
		}
		if(Math.abs(coDriver.getX(Hand.kRight)) > 0){
			robot.turret.setState(Turret.ControlState.Manual);
			robot.turret.setPercentVBus(coDriver.getX(Hand.kRight)*0.3);
		}else{
			if(robot.turret.getCurrentState() == Turret.ControlState.Manual){
				robot.turret.lock();
			}
		}
		outputAllToSmartDashboard();
	}
}

