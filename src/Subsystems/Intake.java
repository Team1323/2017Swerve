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
	public CANTalon intakeMotor;
    public Pidgeon pidgey;
	public Intake(){
		intakeMotor = new CANTalon(Ports.INTAKE_MOTOR);
		intakeMotor.changeControlMode(TalonControlMode.PercentVbus);
		intakeMotor.reverseOutput(false);
		intakeMotor.setVoltageRampRate(24);
		pidgey = new Pidgeon(intakeMotor);
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
	
	private final Loop pidgeonLoop = new Loop(){
		@Override
		public void onStart(){
			
		}
		@Override
		public void onLoop(){
			pidgey.update();
		}
		@Override
		public void onStop(){
			
		}
	};
	public Loop getPidgeonLoop(){
		return pidgeonLoop;
	}
	@Override
	public synchronized void stop(){
		intakeMotor.set(0);
	}
	@Override
	public synchronized void zeroSensors(){
		pidgey.setAngle(0.0);
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber(" Intake Current ", intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Voltage", intakeMotor.getOutputVoltage());
		
		SmartDashboard.putNumber(" Heading Angle ", pidgey.getAngle());
		SmartDashboard.putNumber(" Pigeon Rate ", pidgey.getAngularRate());
		SmartDashboard.putBoolean(" Pigeon Good ", pidgey.isGood());
	}
}
