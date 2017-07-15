package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.PigeonState;

import Loops.Loop;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem{

	private static Intake instance = new Intake();
	private CANTalon intakeMotor;
	public CANTalon getTalon(){
		return intakeMotor;
	}
	public Intake(){
		intakeMotor = new CANTalon(Ports.INTAKE_MOTOR);
		intakeMotor.changeControlMode(TalonControlMode.PercentVbus);
		intakeMotor.reverseOutput(false);
		intakeMotor.setVoltageRampRate(24);
	}
	public static Intake getInstance(){
		return instance;
	}
	
	public void intakeForward(){
		intakeMotor.set(-0.7); 
	}
	public void intakeReverse(){
		intakeMotor.set(0.7);
	}
	@Override
	public synchronized void stop(){
		intakeMotor.set(0);
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
