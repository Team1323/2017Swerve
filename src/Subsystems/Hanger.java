package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger extends Subsystem{
	private CANTalon motor;
	private static Hanger instance = new Hanger();
	public static Hanger getInstance(){
		return instance;
	}
	public Hanger(){
		motor = new CANTalon(Ports.HANGER);
		motor.reverseOutput(true);
		motor.enableBrakeMode(true);
	}
	
	public enum State{
		STARTING_HANG, HANGING, HANG_DETECTED, OFF
	}
	private State currentState = State.OFF;
	public State getState(){
		return currentState;
	}
	public void setState(State newState){
		currentState = newState;
	}
	
	public void update(){
		switch(currentState){
			case OFF:
				stop();
				SmartDashboard.putString(" Hanger Status ", "Off");
				break;
			case STARTING_HANG:
				on();
				SmartDashboard.putString(" Hanger Status ", "Starting Hang");
				setState(State.HANGING);
				break;
			case HANGING:
				if(motor.getOutputCurrent() > Constants.HANG_CURRENT_THRESHOLD){
					setState(State.HANG_DETECTED);
				}
				SmartDashboard.putString(" Hanger Status ", "Hanging Waiting");
				break;
			case HANG_DETECTED:
				stop();
				SmartDashboard.putString(" Hanger Status ", "Hang Complete");
		}
	}
	
	private final Loop hangerLoop = new Loop(){
		@Override
		public void onStart(){
			stop();
		}
		@Override
		public void onLoop(){
			update();
		}
		@Override
		public void onStop(){
			stop();
		}
	};
	public Loop getLoop(){
		return hangerLoop;
	}
	
	public void startHang(){
		setState(State.STARTING_HANG);
	}
	
	public void on(){
		motor.changeControlMode(TalonControlMode.Current);
		motor.set(Constants.HANG_CURRENT);
	}
	@Override
	public synchronized void stop(){
		motor.changeControlMode(TalonControlMode.PercentVbus);
		motor.set(0);
	}
	@Override
	public synchronized void zeroSensors(){
		//zilch
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Hanger Current", motor.getOutputCurrent());
		SmartDashboard.putNumber("Hanger Voltage", motor.getOutputVoltage());
	}
}
