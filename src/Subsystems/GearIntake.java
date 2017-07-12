package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import Loops.Loop;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake extends Subsystem{
	private CANTalon intake;
	private Solenoid cylinder;
	public GearIntake(){
		intake = new CANTalon(Ports.GEAR_INTAKE);
		intake.enableBrakeMode(false);
		intake.changeControlMode(TalonControlMode.Current);
		intake.setPID(0.06, 0.00, 0, 0.08, 0, 0.0, 0);
		intake.setProfile(0);
		intake.reverseOutput(true);
		intake.setCloseLoopRampRate(12);
		intake.setCurrentLimit(15);
		intake.EnableCurrentLimit(true);
		cylinder = new Solenoid(20, Ports.GEAR_INTAKE_ARM);
	}
	private static GearIntake instance = new GearIntake();
	public static GearIntake getInstance(){
		return instance;
	}
	boolean extended = false;
	boolean hasGear = false;
	boolean isScoring = false;
	
	public enum State{
		OFF,
		TUCKED, REVERSED,
		EXTENDED_OFF, EXTENDED_INTAKING, EXTENDED_HOLDING, 
		RETRACTED_OFF, RETRACTED_HOLDING,
		SCORING
	}
	private State currentState = State.TUCKED;
	public State getState(){
		return currentState;
	}
	public void setState(State newState){
		currentState = newState;
	}
	
	public void update(){
		switch(currentState){
			case EXTENDED_OFF:
				stopRoller();
				if(!extended)extendCylinder();
				SmartDashboard.putString("Gear Intake Status", "EXTENDED OFF");
				break;
			case EXTENDED_INTAKING:
				forward();
				if(!extended)extendCylinder();
				if(intake.getOutputCurrent() > Constants.GEAR_DETECT_CURRENT){
					hasGear = true;
					setState(State.EXTENDED_HOLDING);
				}
				SmartDashboard.putString("Gear Intake Status", "EXTENDED INTAKING");
				break;
			case EXTENDED_HOLDING:
				hold();
				if(intake.getOutputCurrent() < Constants.GEAR_PRESENT_CURRENT){
					hasGear = false;
				}
				SmartDashboard.putString("Gear Intake Status", "EXTENDED HOLDING");
				break;
			case RETRACTED_OFF:
				stopRoller();
				retractCylinder();
				SmartDashboard.putString("Gear Intake Status", "RETRACTED OFF");
				break;
			case RETRACTED_HOLDING:
				retractCylinder();
				hold();
				if(intake.getOutputCurrent() > Constants.GEAR_PRESENT_CURRENT){
					hasGear = false;
				}
				SmartDashboard.putString("Gear Intake Status", "RETRACTED HOLDING");
				break;
			case REVERSED:
				reverse();
				SmartDashboard.putString("Gear Intake Status", "REVERSED");
			case SCORING:
				SmartDashboard.putString("Gear Intake Status", "SCORING");
				break;
			case OFF:
				stopRoller();
				SmartDashboard.putString("Gear Intake Status", "OFF");
			default:
				break;
		}
	}
	
	private final Loop gearIntakeLoop = new Loop(){
		@Override
		public void onStart(){
			
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
		return gearIntakeLoop;
	}
	
	private void extendCylinder(){
		cylinder.set(false);
		extended = !cylinder.get();
	}
	private void retractCylinder(){
		cylinder.set(true);
		extended = !cylinder.get();
	}
	private void forward(){
		intake.changeControlMode(TalonControlMode.PercentVbus);
		intake.set(-0.75);
	}
	private void hold(){
		intake.changeControlMode(TalonControlMode.PercentVbus);
		intake.set(-0.27);
	}
	private void reverse(){
		intake.changeControlMode(TalonControlMode.Current);
		intake.set(-12);
	}
	private void stopRoller(){
		intake.changeControlMode(TalonControlMode.PercentVbus);
		intake.set(0);
	}
	
	public void intakeGear(){
		setState(State.EXTENDED_INTAKING);
	}
	public void retract(){
		if(hasGear){
			setState(State.RETRACTED_HOLDING);
		}else{
			setState(State.RETRACTED_OFF);
		}
	}
	public void coDriverStop(){
		if(!hasGear){
			if(extended){
				setState(State.EXTENDED_OFF);
			}else{
				setState(State.RETRACTED_OFF);
			}
		}
	}
	
	public void score(){
		if(!isScoring){
			ScoreGear scoreAction = new ScoreGear();
			scoreAction.start();
		}
	}
	public class ScoreGear extends Thread{
		public void run(){
			isScoring = true;
			stopRoller();
			Timer.delay(0.1);
			reverse();
			Timer.delay(0.25);
			extendCylinder();
			Timer.delay(0.75);
			setState(GearIntake.State.EXTENDED_OFF);
			isScoring = false;
		}
	}
	
	@Override
	public synchronized void stop(){
		setState(State.OFF);
	}
	@Override
	public synchronized void zeroSensors(){
		//nada
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Gear Intake Voltage" , intake.getOutputVoltage());
		SmartDashboard.putNumber("Gear Intake Current", intake.getOutputCurrent());
	}
}
