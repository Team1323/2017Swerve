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
	private double currentAngle = 0.0;
    private PigeonImu _pidgey;
    boolean angleIsGood = false;
    double currentAngularRate = 0.0;
    public static int ZERO = 0;
    
    public enum AnglePresets{
		ZERO,NINETY,ONE_EIGHTY, TWO_SEVENTY
	}
	public Intake(){
		intakeMotor = new CANTalon(Ports.INTAKE_MOTOR);
		intakeMotor.changeControlMode(TalonControlMode.PercentVbus);
		//intakeMotor.setPID(/*0.01*/0.02, 0.00, 0, 0.0175, 0, 0.0, 0);
		intakeMotor.reverseOutput(false);
		//intakeMotor.setCloseLoopRampRate(24);
		intakeMotor.setVoltageRampRate(24);
		try{
			_pidgey = new PigeonImu(intakeMotor);
		}catch(Exception e){
			System.out.println(e);
		}
	}
	public static Intake getInstance(){
		return instance;
	}
	public void pidgeonUpdate(){
		try{
			PigeonImu.GeneralStatus genStatus = new PigeonImu.GeneralStatus();
			PigeonImu.FusionStatus fusionStatus = new PigeonImu.FusionStatus();
			double [] xyz_dps = new double [3];
			currentAngle = -_pidgey.GetFusedHeading(fusionStatus);
			_pidgey.GetGeneralStatus(genStatus);
			_pidgey.GetRawGyro(xyz_dps);
			angleIsGood = (_pidgey.GetState() == PigeonState.Ready) ? true : false;
			currentAngularRate = -xyz_dps[2];
			
			short [] ba_xyz = new short [3];
			_pidgey.GetBiasedAccelerometer(ba_xyz);
			//SmartDashboard.putNumber("AccX", ba_xyz[0]);
			//SmartDashboard.putNumber("AccY", ba_xyz[1]);
			//SmartDashboard.putNumber("AccZ", ba_xyz[2]);
			
		}catch(Exception e){
			System.out.println(e);
		}
	}
	public boolean pidgeyGood(){
		return angleIsGood;
	}
	public double getCurrentAngle(){
		return currentAngle;
		
	}
	public double getCurrentAngularRate(){
		return currentAngularRate;
	}
	public void intakeForward(){
		intakeMotor.set(-0.7); 
	}
	public void reducedForward(){
		intakeMotor.set(-0.7);
	}
	public void intakeReverse(){
		intakeMotor.set(0.7);
	}
	public void setPidgeonAngle(double i){
		if(i != 0.0){
			_pidgey.SetFusedHeading(360.0-i);
		}else{
			_pidgey.SetFusedHeading(i);
		}
	}
	private final Loop intakeLoop = new Loop(){
		@Override
		public void onStart(){
			
		}
		@Override
		public void onLoop(){
			pidgeonUpdate();
		}
		@Override
		public void onStop(){
			
		}
	};
	public Loop getLoop(){
		return intakeLoop;
	}
	@Override
	public synchronized void stop(){
		intakeMotor.set(0);
	}
	@Override
	public synchronized void zeroSensors(){
		setPidgeonAngle(0);
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber(" Intake Current ", intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Error", intakeMotor.getSetpoint()-intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake Voltage", intakeMotor.getOutputVoltage());
		
		SmartDashboard.putNumber(" Heading Angle ", currentAngle);
		SmartDashboard.putNumber(" Pigeon Rate ", currentAngularRate);
		SmartDashboard.putBoolean(" Pigeon Good ", angleIsGood);
	}
}
