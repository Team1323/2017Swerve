package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Loops.Loop;
import Utilities.CircularBuffer;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem{
	private CANTalon master, slave;
	public Shooter(){
		master = new CANTalon(Ports.SHOOTER_MOTOR_MASTER);
		master.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		master.reverseSensor(true);
		master.reverseOutput(false);
		master.configNominalOutputVoltage(+0f, -0f);
		master.configPeakOutputVoltage(12f, -0f);
		master.setAllowableClosedLoopErr(0);
		master.changeControlMode(TalonControlMode.PercentVbus);
		master.set(0);
		master.setPID(4.0, 0.00, 40, 0.027, 0, 0.0, 0);
		//master.setPID(0.08, 0.0, 0.0, 0.027, 0, 60.0, 0);
		master.setPID(0.0, 0.00, 0, 0.027, 0, 720.0, 1);
		master.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 2);
		master.setStatusFrameRateMs(CANTalon.StatusFrameRate.AnalogTempVbat, 2);
		master.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_10Ms);
		master.SetVelocityMeasurementWindow(32);
		master.setNominalClosedLoopVoltage(12);
		master.enableBrakeMode(false);
		
		if(master.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent){
			DriverStation.reportError("Could not detect shooter encoder!", false);
		}
		
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
		
		mControlMethod = ControlMethod.OPEN_LOOP;
	}
	private static Shooter instance = new Shooter();
	public static Shooter getInstance(){
		return instance;
	}
	
	public enum ControlMethod {
        OPEN_LOOP, // open loop voltage control for running the climber
        SPIN_UP, // PIDF to desired RPM
        HOLD_WHEN_READY, // calculate average kF
        HOLD, // switch to pure kF control
    }
	private ControlMethod mControlMethod;
    private double mSetpointRpm;
    private double mLastRpmSpeed;
    
    private CircularBuffer mKfEstimator = new CircularBuffer(20);
    
 // Used for transitioning from spin-up to hold loop.
    private boolean mOnTarget = false;
    private double mOnTargetStartTime = Double.POSITIVE_INFINITY;
    
    private final Loop shooterLoop = new Loop(){
    	@Override
        public void onStart() {
            synchronized (Shooter.this) {
                mControlMethod = ControlMethod.OPEN_LOOP;
                mKfEstimator.clear();
                mOnTarget = false;
                mOnTargetStartTime = Double.POSITIVE_INFINITY;
            }
        }
    	
    	@Override
        public void onLoop() {
            synchronized (Shooter.this) {
                if (mControlMethod != ControlMethod.OPEN_LOOP) {
                    handleClosedLoop(Timer.getFPGATimestamp());
                } else {
                    // Reset all state.
                    mKfEstimator.clear();
                    mOnTarget = false;
                    mOnTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }
        }
    	
    	@Override
    	public void onStop(){
    		stop();
    	}
    };
    
    public Loop getLoop(){
    	return shooterLoop;
    }
    
    public synchronized void setSpinUp(double setpointRpm) {
        if (mControlMethod != ControlMethod.SPIN_UP) {
            configureForSpinUp();
        }
        mSetpointRpm = setpointRpm;
        master.set(mSetpointRpm);
    }
    
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if (mControlMethod == ControlMethod.OPEN_LOOP || mControlMethod == ControlMethod.SPIN_UP) {
            configureForHoldWhenReady();
        }
        mSetpointRpm = setpointRpm;
        master.set(mSetpointRpm);
    }
    
    private void configureForSpinUp() {
        mControlMethod = ControlMethod.SPIN_UP;
        master.changeControlMode(CANTalon.TalonControlMode.Speed);
        //master.DisableNominalClosedLoopVoltage();
        master.setProfile(0);
    }
    
    private void configureForHoldWhenReady() {
        mControlMethod = ControlMethod.HOLD_WHEN_READY;
        master.changeControlMode(CANTalon.TalonControlMode.Speed);
        //master.DisableNominalClosedLoopVoltage();
        master.setProfile(0);
    }
    
    private void configureForHold() {
        mControlMethod = ControlMethod.HOLD;
        master.changeControlMode(CANTalon.TalonControlMode.Speed);
        master.setProfile(1);
        master.setNominalClosedLoopVoltage(12.0);
        master.setF(mKfEstimator.getAverage());
        System.out.println(mKfEstimator.getAverage());
    }
    
    private void resetHold() {
        mKfEstimator.clear();
        mOnTarget = false;
    }
    
    private double estimateKf(double rpm, double voltage) {
        final double speed_in_ticks_per_100ms = 4096.0 / 600.0 * rpm;
        final double output = 1023.0 / 12.0 * voltage;
        System.out.println(output/speed_in_ticks_per_100ms);
        return output / speed_in_ticks_per_100ms;
    }
    
    private void handleClosedLoop(double timestamp) {
        final double speed = getSpeed();
        final double voltage = master.getOutputVoltage();
        mLastRpmSpeed = speed;

        // See if we should be spinning up or holding.
        if (mControlMethod == ControlMethod.SPIN_UP) {
            master.set(mSetpointRpm);
            resetHold();
            
        } else if (mControlMethod == ControlMethod.HOLD_WHEN_READY) {
            final double abs_error = Math.abs(speed - mSetpointRpm);
            final boolean on_target_now = mOnTarget ? abs_error < 150 :
            	abs_error < 50;
            if (on_target_now && !mOnTarget) {
                // First cycle on target.
                mOnTargetStartTime = timestamp;
                mOnTarget = true;
            } else if (!on_target_now) {
                resetHold();
            }

            if (mOnTarget) {
                // Update Kv.
                mKfEstimator.addValue(estimateKf(speed, voltage));
            }
            if (mKfEstimator.getNumValues() >= 20) {
                configureForHold();
            } else {
                master.set(mSetpointRpm);
            }
        }
        // No else because we may have changed control methods above.
        if (mControlMethod == ControlMethod.HOLD) {
            // Update Kv if we exceed our target velocity. As the system heats up, drag is reduced.
            if (speed > mSetpointRpm) {
                mKfEstimator.addValue(estimateKf(speed, voltage));
                master.setF(mKfEstimator.getAverage());
            }
        }
    }
	
	public void setSpeed(double speed){
		master.changeControlMode(TalonControlMode.Speed);
		master.set(speed);
	}
	public void setOpenLoop(double power){
		mControlMethod = ControlMethod.OPEN_LOOP;
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
		setOpenLoop(0);
		mSetpointRpm = 0.0;
	}
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Shooter Voltage", master.getOutputVoltage());
		SmartDashboard.putNumber("Shooter Current", master.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Shooter Error", getError());
		SmartDashboard.putNumber("SHOOTER_SPEED", getSpeed());
    	SmartDashboard.putNumber("SHOOTER_SPEED_GRAPH", getSpeed());
	}
}
