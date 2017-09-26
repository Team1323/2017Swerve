package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem{

	private static Intake instance = new Intake();
	private CANTalon intakeMotor;
	public CANTalon getTalon(){
		return intakeMotor;
	}
	boolean isForward = false;
	boolean isReversed = false;
	public Intake(){
		intakeMotor = new CANTalon(Ports.INTAKE_MOTOR);
		intakeMotor.changeControlMode(TalonControlMode.PercentVbus);
		intakeMotor.reverseOutput(false);
		intakeMotor.setVoltageRampRate(24);
		intakeMotor.setCurrentLimit(25);
		intakeMotor.EnableCurrentLimit(true);
	}
	public static Intake getInstance(){
		return instance;
	}
	
	public void intakeForward(){
		intakeMotor.set(-1.0); 
		isForward = true;
	}
	public void intakeReverse(){
		intakeMotor.set(1.0);
		isReversed = true;
	}
	public void toggleForward(){
		if(isForward){
			stop();
		}else{
			intakeForward();
		}
	}
	public void toggleReverse(){
		if(isReversed){
			stop();
		}else{
			intakeReverse();
		}
	}
	@Override
	public synchronized void stop(){
		intakeMotor.set(0);
		isForward = false;
		isReversed = false;
	}
	@Override
	public synchronized void zeroSensors(){
		
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber(" Intake Current ", intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Voltage", intakeMotor.getOutputVoltage());
	}
}
