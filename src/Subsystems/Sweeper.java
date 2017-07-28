package Subsystems;

import com.ctre.CANTalon;

import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sweeper extends Subsystem{
	public CANTalon sweeperArm;
	public CANTalon sweeperRoller;
	public Sweeper(){
		sweeperArm = new CANTalon(Ports.SWEEPER);
		sweeperArm.setVoltageRampRate(48);
		sweeperRoller = new CANTalon(Ports.SWEEPER_ROLLER);
		sweeperRoller.setCurrentLimit(50);
		sweeperRoller.EnableCurrentLimit(true);
	}
	
	private static Sweeper instance = new Sweeper();
	public static Sweeper getInstance(){
		return instance;
	}
	
	public void armForward(){
		sweeperArm.set(Constants.SWEEPER_FORWARD);
	}
	public void rollerForward(){
		sweeperRoller.set(Constants.SWEEPER_ROLLER_FORWARD);
	}
	public void rollerReverse(){
		sweeperRoller.set(Constants.SWEEPER_ROLLER_REVERSE);
	}
	public void startSweeper(){
		SweeperSequence sequence = new SweeperSequence();
		sequence.start();
	}
	public class SweeperSequence extends Thread{
		public void run(){
			rollerForward();
			Timer.delay(0.25);
			armForward();
		}
	}
	@Override
	public synchronized void stop(){
		sweeperArm.set(0);
		sweeperRoller.set(0);
	}
	@Override
	public synchronized void zeroSensors(){
		
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Sweeper Rotor Current", sweeperArm.getOutputCurrent());
		SmartDashboard.putNumber("Sweeper Roller Current", sweeperRoller.getOutputCurrent());
		SmartDashboard.putNumber("Sweeper Rotor Voltage", sweeperArm.getOutputVoltage());
		SmartDashboard.putNumber("Sweeper Roller Voltage", sweeperRoller.getOutputVoltage());
	}
}
