package Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hanger extends Subsystem{
	private TalonSRX motor;
	public TalonSRX getTalon(){
		return motor;
	}
	private static Hanger instance = new Hanger();
	public static Hanger getInstance(){
		return instance;
	}
	public Hanger(){
		motor = new TalonSRX(Ports.HANGER);
		motor.setInverted(false);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.configPeakCurrentLimit(70, 10);
		motor.enableCurrentLimit(true);
	}
	
	public enum State{
		STARTING_HANG, HANGING, HANG_DETECTED, OFF, WEAK_HANG
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
				/*if(motor.getOutputCurrent() > Constants.HANG_STOP_CURRENT){
					setState(State.HANG_DETECTED);
				}*/
				SmartDashboard.putString(" Hanger Status ", "Hanging Waiting");
				break;
			case HANG_DETECTED:
				stop();
				SmartDashboard.putString(" Hanger Status ", "Hang Complete");
				break;
			case WEAK_HANG:
				motor.set(ControlMode.PercentOutput, -4.0/12.0);
				break;
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
		motor.set(ControlMode.PercentOutput, Constants.HANG_POWER);
	}
	
	public boolean isHanging(){
		return (currentState == State.HANGING) && (motor.getOutputCurrent() > Constants.HANGING_DETECT_CURRENT);
	}
	
	@Override
	public synchronized void stop(){
		setState(State.OFF);
		motor.set(ControlMode.PercentOutput, 0);
	}
	@Override
	public synchronized void zeroSensors(){
		//zilch
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Hanger Current", motor.getOutputCurrent());
		SmartDashboard.putNumber("Hanger Voltage", motor.getMotorOutputVoltage());
	}
}
