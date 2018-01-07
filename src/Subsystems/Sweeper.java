package Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sweeper extends Subsystem{
	public TalonSRX sweeperArm;
	public TalonSRX sweeperRoller;
	public Sweeper(){
		sweeperArm = new TalonSRX(Ports.SWEEPER);
		sweeperArm.configOpenloopRamp(0.25, 10);
		sweeperRoller = new TalonSRX(Ports.SWEEPER_ROLLER);
		sweeperRoller.configPeakCurrentLimit(50, 10);
		sweeperRoller.enableCurrentLimit(true);
	}
	
	private static Sweeper instance = new Sweeper();
	public static Sweeper getInstance(){
		return instance;
	}
	public boolean isInThread = false;
	private boolean isFeeding = false;
	public boolean isFeeding(){
		return isFeeding;
	}
	
	public void armForward(){
		sweeperArm.set(ControlMode.PercentOutput, Constants.SWEEPER_FORWARD);
	}
	public void rollerForward(){
		sweeperRoller.set(ControlMode.PercentOutput, Constants.SWEEPER_ROLLER_FORWARD);
	}
	public void rollerReverse(){
		sweeperRoller.set(ControlMode.PercentOutput, Constants.SWEEPER_ROLLER_REVERSE);
	}
	public void startSweeper(){
		if(!isInThread){
			SweeperSequence sequence = new SweeperSequence();
			sequence.start();
		}
		isFeeding = true;
	}
	public class SweeperSequence extends Thread{
		public void run(){
			isInThread = true;
			rollerForward();
			Timer.delay(0.25);
			armForward();
			isInThread = false;
		}
	}
	@Override
	public synchronized void stop(){
		sweeperArm.set(ControlMode.PercentOutput, 0);
		sweeperRoller.set(ControlMode.PercentOutput, 0);
		isFeeding = false;
	}
	@Override
	public synchronized void zeroSensors(){
		
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Sweeper Rotor Current", sweeperArm.getOutputCurrent());
		SmartDashboard.putNumber("Sweeper Roller Current", sweeperRoller.getOutputCurrent());
		SmartDashboard.putNumber("Sweeper Rotor Voltage", sweeperArm.getMotorOutputVoltage());
		SmartDashboard.putNumber("Sweeper Roller Voltage", sweeperRoller.getMotorOutputVoltage());
	}
}
