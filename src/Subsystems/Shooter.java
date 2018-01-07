package Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Loops.Loop;
import Utilities.CircularBuffer;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem{
	private TalonSRX master, slave;
	public Shooter(){
		master = new TalonSRX(Ports.SHOOTER_MOTOR_MASTER);
		master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		master.setSensorPhase(true);
		master.setInverted(false);
		master.configNominalOutputForward(0.0, 10);
		master.configNominalOutputReverse(0.0, 10);
		master.configPeakOutputForward(1.0, 10);
		master.configPeakOutputReverse(0.0, 10);
		master.configAllowableClosedloopError(0, 0, 10);
		master.set(ControlMode.PercentOutput, 0);
		/*master.setPID(4.0, 0.00, 40, 0.027, 0, 0.0, 0);
		master.setPID(0.0, 0.00, 0, 0.027, 0, 720.0, 1);*/
		master.selectProfileSlot(0, 0);
		master.config_kP(0, 4.0, 10);
		master.config_kI(0, 0.0, 10);
		master.config_kD(0, 40, 10);
		master.config_kF(0, 0.027, 10);
		master.setStatusFramePeriod(StatusFrame.Status_1_General, 2, 10);
		master.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 2, 10);
		master.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
		master.configVelocityMeasurementWindow(32, 10);
		master.enableVoltageCompensation(true);
		//master.setNominalClosedLoopVoltage(12);
		master.setNeutralMode(NeutralMode.Coast);
		
		slave = new TalonSRX(Ports.SHOOTER_MOTOR_SLAVE);
		slave.set(ControlMode.Follower, Ports.SHOOTER_MOTOR_MASTER);
		slave.setInverted(true);
		slave.setNeutralMode(NeutralMode.Coast);
		slave.enableVoltageCompensation(true);
		
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
        master.set(ControlMode.Velocity, mSetpointRpm);
    }
    
    public synchronized void setHoldWhenReady(double setpointRpm) {
        if (mControlMethod == ControlMethod.OPEN_LOOP || mControlMethod == ControlMethod.SPIN_UP) {
            configureForHoldWhenReady();
        }
        mSetpointRpm = setpointRpm;
        master.set(ControlMode.Velocity, mSetpointRpm);
    }
    
    private void configureForSpinUp() {
        mControlMethod = ControlMethod.SPIN_UP;
        //master.DisableNominalClosedLoopVoltage();
        master.selectProfileSlot(0, 0);
    }
    
    private void configureForHoldWhenReady() {
        mControlMethod = ControlMethod.HOLD_WHEN_READY;
        //master.DisableNominalClosedLoopVoltage();
        master.selectProfileSlot(0, 0);
    }
    
    private void configureForHold() {
        mControlMethod = ControlMethod.HOLD;
        /*master.setProfile(1);
        master.setNominalClosedLoopVoltage(12.0);
        master.setF(mKfEstimator.getAverage());*/
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
        final double voltage = master.getMotorOutputVoltage();
        mLastRpmSpeed = speed;

        // See if we should be spinning up or holding.
        if (mControlMethod == ControlMethod.SPIN_UP) {
            master.set(ControlMode.Velocity, mSetpointRpm);
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
                master.set(ControlMode.Velocity, mSetpointRpm);
            }
        }
        // No else because we may have changed control methods above.
        if (mControlMethod == ControlMethod.HOLD) {
            // Update Kv if we exceed our target velocity. As the system heats up, drag is reduced.
            if (speed > mSetpointRpm) {
                mKfEstimator.addValue(estimateKf(speed, voltage));
                //master.setF(mKfEstimator.getAverage());
            }
        }
    }
	
	public void setSpeed(double speed){
		master.set(ControlMode.Velocity, speed);
	}
	public void setOpenLoop(double power){
		mControlMethod = ControlMethod.OPEN_LOOP;
		master.set(ControlMode.PercentOutput, power);
	}
	public double getSpeed(){
		return master.getSelectedSensorVelocity(0);
	}
	public double getGoal(){
		return mSetpointRpm;
	}
	public double getError(){
		return Math.abs(getGoal() - getSpeed());
	}
	public boolean onTarget(){
		return ((master.getControlMode() == ControlMode.Velocity)
				&& (getError() < Constants.SHOOTER_ALLOWABLE_ERROR));
	}
	public boolean isShooting(){
		return master.getControlMode() == ControlMode.Velocity
				&& getGoal() != 0;
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
		SmartDashboard.putNumber("Shooter Voltage", master.getMotorOutputVoltage());
		SmartDashboard.putNumber("Shooter Current", master.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Shooter Error", getError());
		SmartDashboard.putNumber("SHOOTER_SPEED", getSpeed());
    	SmartDashboard.putNumber("SHOOTER_SPEED_GRAPH", getSpeed());
	}
}
