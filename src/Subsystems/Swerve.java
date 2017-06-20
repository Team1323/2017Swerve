package Subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends Subsystem{
	private static Swerve instance = new Swerve();
	private Intake intake = Intake.getInstance();
	private double xInput;
	private double yInput;
	private double rotateInput;
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
	public Swerve(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2,Constants.FRONT_LEFT_TURN_OFFSET);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1,Constants.FRONT_RIGHT_TURN_OFFSET);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3,Constants.REAR_LEFT_TURN_OFFSET);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4,Constants.REAR_RIGHT_TURN_OFFSET);
	}
	public static Swerve getInstance(){
        return instance;
    }
	public class SwerveDriveModule extends Subsystem{
		private CANTalon rotationMotor;
		public CANTalon driveMotor;
		private int moduleID;
		private int absolutePosition;
		private double offSet = 0.0;
		public SwerveDriveModule(int rotationMotorPort, int driveMotorPort,int moduleNum,double _offSet){
			rotationMotor = new CANTalon(rotationMotorPort);
			rotationMotor.setPID(2, 0.0, 30, 0.0, 0, 0.0, 0);
			driveMotor = new CANTalon(driveMotorPort);	    	
//	    	driveMotor.changeControlMode(TalonControlMode.);
//	    	driveMotor.set(driveMotor.getPosition());
			loadProperties();
			moduleID = moduleNum;  
			offSet = _offSet;
			double targetAngle = 0;
		}
		public final void loadProperties(){
	    	absolutePosition = rotationMotor.getPulseWidthPosition() & 0xFFF;
	    	rotationMotor.setEncPosition(absolutePosition);
	    	rotationMotor.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
	    	rotationMotor.reverseSensor(false);
	    	rotationMotor.reverseOutput(true);
	    	rotationMotor.configPotentiometerTurns(360);
	    	rotationMotor.configNominalOutputVoltage(+0f, -0f);
	    	rotationMotor.configPeakOutputVoltage(+7f, -7f);
	    	rotationMotor.setAllowableClosedLoopErr(0); 
	    	rotationMotor.changeControlMode(TalonControlMode.Position);
	    	rotationMotor.setPID(5.0, 0.0, 40.0, 0.00, 0, 0.0, 0);
	    	rotationMotor.setProfile(0);
	    	rotationMotor.set(rotationMotor.get());
	    	driveMotor.setEncPosition(0);
	    	driveMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    	driveMotor.configEncoderCodesPerRev(360);
	    	driveMotor.configNominalOutputVoltage(+0f, -0f);
	    	driveMotor.configPeakOutputVoltage(+12f, -0f);
	    	driveMotor.setAllowableClosedLoopErr(0);
	    	driveMotor.changeControlMode(TalonControlMode.PercentVbus);
	    	driveMotor.reverseOutput(false);
		}
		public double getModuleAngle(){
			return Util.boundAngle0to360Degrees(rotationMotor.get() - offSet);
		}
		public void setModuleAngle(double goalAngle){
			double newAngle = Util.continousAngle(goalAngle-(360-offSet),getModuleAngle());
		}
		public void setDriveSpeed(double power){
			driveMotor.set(power);
		}
		@Override
		public synchronized void stop(){
			rotationMotor.set(rotationMotor.get());
			driveMotor.set(0);
		}
		@Override
		public synchronized void zeroSensors(){
			driveMotor.setEncPosition(0);
		}
		@Override
		public void outputToSmartDashboard(){
			SmartDashboard.putNumber("Module " + Integer.toString(moduleID) + " Angle", getModuleAngle());
		}
	}
	
	public void sendInput(double x, double y, double rotate, boolean robotCentric){
		double angle = intake.getCurrentAngle()/180.0*Math.PI;
		if(robotCentric){
			xInput = x;
			yInput = y;
		}
		else{
			double tmp = (y* Math.cos(angle)) + (x * Math.sin(angle));
			xInput = (-y * Math.sin(angle)) + (x * Math.cos(angle));
			yInput = tmp;			
		}
		rotateInput = rotate;
	}
	public void update(){
		double A = xInput - rotateInput * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
	    double B = xInput + rotateInput * (Constants.WHEELBASE_LENGTH / Constants.SWERVE_R);
	    double C = yInput - rotateInput * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
	    double D = yInput + rotateInput * (Constants.WHEELBASE_WIDTH / Constants.SWERVE_R);
	    
	    double frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
	    double frontLeftWheelSpeed  = Math.sqrt((B * B) + (D * D));
	    double rearLeftWheelSpeed   = Math.sqrt((A * A) + (D * D));
	    double rearRightWheelSpeed  = Math.sqrt((A * A) + (C * C));
	    double max = frontRightWheelSpeed;
	    max = Util.normalize(max, frontLeftWheelSpeed);
	    max = Util.normalize(max, rearLeftWheelSpeed);
	    max = Util.normalize(max, rearRightWheelSpeed);
	    if(max > 1.0){
	    	frontRightWheelSpeed /= max;
	        frontLeftWheelSpeed /= max;
	        rearLeftWheelSpeed /= max;
	        rearRightWheelSpeed /= max;
	    }
	    
	    double frontRightSteeringAngle = Math.atan2(B, C)*180/Math.PI; 
	    double frontLeftSteeringAngle = Math.atan2(B, D)*180/Math.PI;
	    double rearLeftSteeringAngle = Math.atan2(A, D)*180/Math.PI;
	    double rearRightSteeringAngle = Math.atan2(A, C)*180/Math.PI;
	    
	    frontRight.setModuleAngle(frontRightSteeringAngle);
	    frontLeft.setModuleAngle(frontLeftSteeringAngle);
	    rearLeft.setModuleAngle(rearLeftSteeringAngle);
	    rearRight.setModuleAngle(rearRightSteeringAngle);
	    
	    frontRight.setDriveSpeed(frontRightWheelSpeed);
	    frontLeft.setDriveSpeed(frontLeftWheelSpeed);
	    rearLeft.setDriveSpeed(rearLeftWheelSpeed);
	    rearRight.setDriveSpeed(rearRightWheelSpeed);
	}
	@Override
	public synchronized void stop(){
		sendInput(0.0, 0.0, 0.0, false);
		frontRight.stop();
		frontLeft.stop();
		rearLeft.stop();
		rearRight.stop();
	}
	@Override
	public synchronized void zeroSensors(){
		frontRight.zeroSensors();
		frontLeft.zeroSensors();
		rearLeft.zeroSensors();
		rearRight.zeroSensors();
	}
	@Override
	public void outputToSmartDashboard(){
		frontRight.outputToSmartDashboard();
		frontLeft.outputToSmartDashboard();
		rearLeft.outputToSmartDashboard();
		rearRight.outputToSmartDashboard();
	}
}