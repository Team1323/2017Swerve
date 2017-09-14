package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem{
	private CANTalon master, slave;
	public Shooter(){
		master = new CANTalon(Ports.SHOOTER_MOTOR_MASTER);
		master.setEncPosition(0);
		master.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		master.reverseSensor(true);
		master.reverseOutput(false);
		master.configNominalOutputVoltage(+0f, -0f);
		master.configPeakOutputVoltage(12f, -0f);
		master.setAllowableClosedLoopErr(0);
		master.changeControlMode(TalonControlMode.Speed);
		master.set(0);
		master.setPID(4.0, 0.00, 40, 0.027, 0, 0.0, 0);
		master.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 2);
		master.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_10Ms);
		master.SetVelocityMeasurementWindow(32);
		master.setNominalClosedLoopVoltage(12);
		master.enableBrakeMode(false);
		
		slave = new CANTalon(Ports.SHOOTER_MOTOR_SLAVE);
		slave.changeControlMode(TalonControlMode.Follower);
		slave.set(Ports.SHOOTER_MOTOR_MASTER);
		slave.reverseOutput(true);
		slave.enableBrakeMode(false);
		slave.configNominalOutputVoltage(+0f, -0f);
		slave.configPeakOutputVoltage(12f, -0f);
		slave.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 2);
		slave.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_10Ms);
		slave.SetVelocityMeasurementWindow(32);
		slave.setNominalClosedLoopVoltage(12);
	}
	private static Shooter instance = new Shooter();
	public static Shooter getInstance(){
		return instance;
	}
	
	public void setSpeed(double speed){
		master.changeControlMode(TalonControlMode.Speed);
		master.set(speed);
	}
	public void setPercentVBus(double power){
		master.changeControlMode(TalonControlMode.PercentVbus);
		master.set(power);
	}
	public double getSpeed(){
		return master.getSpeed();
	}
	public double getGoal(){
		return master.getSetpoint();
	}
	public double getError(){
		return Math.abs(getGoal() - getSpeed());
	}
	public boolean onTarget(){
		return ((master.getControlMode() == CANTalon.TalonControlMode.Speed)
				&& (getError() < Constants.SHOOTER_ALLOWABLE_ERROR));
	}
	
	@Override
	public synchronized void stop(){
		setPercentVBus(0);
	}
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Shooter Voltage", master.getOutputVoltage());
		SmartDashboard.putNumber("Shooter Current", master.getOutputCurrent());
		SmartDashboard.putNumber("Shooter 2 Voltage", slave.getOutputVoltage());
		SmartDashboard.putNumber("Shooter 2 Current", slave.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Shooter Error", getError());
		SmartDashboard.putNumber("SHOOTER_SPEED", getSpeed());
    	SmartDashboard.putNumber("SHOOTER_SPEED_GRAPH", getSpeed());
	}
}
