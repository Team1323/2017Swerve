package Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem{

	private static Intake instance = new Intake();
	private TalonSRX intakeMotor;
	public TalonSRX getTalon(){
		return intakeMotor;
	}
	boolean isForward = false;
	boolean isReversed = false;
	public Intake(){
		intakeMotor = new TalonSRX(Ports.INTAKE_MOTOR);
		intakeMotor.setInverted(false);
		intakeMotor.configOpenloopRamp(0.5, 10);
		intakeMotor.configPeakCurrentLimit(30, 10);
		intakeMotor.enableCurrentLimit(true);
	}
	public static Intake getInstance(){
		return instance;
	}
	
	public void intakeForward(){
		intakeMotor.set(ControlMode.PercentOutput, -1.0); 
		isForward = true;
	}
	public void intakeReverse(){
		intakeMotor.set(ControlMode.PercentOutput, 1.0);
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
		intakeMotor.set(ControlMode.PercentOutput, 0);
		isForward = false;
		isReversed = false;
	}
	@Override
	public synchronized void zeroSensors(){
		
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber(" Intake Current ", intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Voltage", intakeMotor.getMotorOutputVoltage());
	}
}
