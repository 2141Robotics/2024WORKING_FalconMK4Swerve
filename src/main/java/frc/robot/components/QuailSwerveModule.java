package frc.robot.components;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.math.Constants;

import com.mineinjava.quail.SwerveModuleBase;
import com.mineinjava.quail.util.geometry.Vec2d;

/**
 * Container for one swerve module. Wraps two falcon500s: one for driving and one for steering.
 * 
 * @author 2141 Spartonics
 */
public class QuailSwerveModule extends SwerveModuleBase
{
	/** The PID id used to determine what PID settings to use. */
	/** The motor controlling the module's movement. */
	public final TalonFX drivingMotor;
	/** The motor controlling the module's rotation. */
	public final TalonFX steeringMotor;
	/** The can coder measuring the module's absolute rotaiton. */
	private final CANcoder canCoder;
	/** The can coder's rotational offset. This value must be manually set through phoenix tuner. */
	public final double canOffset;
	
	/**
	 * @param driveMotor driving motor ID
	 * @param steeringMotor steering motor ID
	 * @param canCoder can coder ID
	 * @param rotationDirection the steering motor's rotational direction, usually perpendicular to the center of the robot
	 * @param canCoderOffset the can coder's rotational offset
	 */

	public QuailSwerveModule(Vec2d position, int driveMotorID, int steeringMotorID, int canCoderID, double canCoderOffset)
	{
		super(position, Constants.steeringRatio, Constants.driveRatio, true);
		this.drivingMotor = new TalonFX(driveMotorID);
		this.steeringMotor = new TalonFX(steeringMotorID);
		this.canCoder = new CANcoder(canCoderID);
		this.canOffset = canCoderOffset;
	}
	
	/**
	 * Configures the motors and sets the steering motor's rotation to zero.
	 */
	public void init()
	{
		// Reset the steering motor.
		
		this.steeringMotor.getConfigurator().apply(new TalonFXConfiguration());
		this.drivingMotor.getConfigurator().apply(new TalonFXConfiguration());
		MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
		encoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
		encoderConfig.MagnetOffset = this.canOffset;
		encoderConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;


		this.canCoder.getConfigurator().apply(encoderConfig);
		// Miscellaneous settings.
		this.steeringMotor.setInverted(true);
		this.drivingMotor.setInverted(true);
		
		
		var slot0Configs = new Slot0Configs();
		slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
		slot0Configs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
		slot0Configs.kV = Constants.PID_SETTINGS[0];
		slot0Configs.kP = Constants.PID_SETTINGS[1];
		slot0Configs.kI = Constants.PID_SETTINGS[2];
		slot0Configs.kD = Constants.PID_SETTINGS[3];
		var talonFXConfigs = new TalonFXConfiguration();

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
		motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
		motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

		this.steeringMotor.getConfigurator().apply(talonFXConfigs);
	
		// PID tune the steering motor.

		this.steeringMotor.getConfigurator().apply(slot0Configs);

    	System.out.println("done config");

		// Reset the motor rotations.
		this.reset();
	}

	/**
	 * Sets steering motor's rotation to zero.
	 */
	public void reset()
	{
		System.out.println("RESET");
		// Set the steering motor's internal rotation to 0.
		double currentPos = this.canCoder.getAbsolutePosition().refresh().getValue();
		// The angle to rotate to face forward.
		double angleToRotate = currentPos > 0.5d ? currentPos - 1d : currentPos;
		// Set the steering motor's rotation.
		this.steeringMotor.setPosition(angleToRotate * 12.8);
		this.currentAngle = angleToRotate * Constants.TWO_PI;
	}

	public Vec2d getCurrentMovement() {
		double angle = this.canCoder.getAbsolutePosition().refresh().getValue() * (2 * Math.PI);
		double velocity = this.drivingMotor.getVelocity().refresh().getValue();
		double rotationsPerSecond = velocity / 6.75;
		double inchesPerSecond = rotationsPerSecond * 4 * Math.PI;

		return new Vec2d(angle, inchesPerSecond, false);
	
	}
	
	
	@Override
	public void setRawAngle(double angle)
	{

		if (angle != angle) {
			return;
		}

		this.steeringMotor.setControl(new PositionDutyCycle(angle * 12.8 / Constants.TWO_PI));
	}

	@Override
	public void setRawSpeed(double speed) {
		this.drivingMotor.set(speed);
	}

	/**
	 * Getter for the drive motor.
	 * 
	 * @return The drive motor
	 */
     	public TalonFX getDriveMotor()
	{
		return this.drivingMotor;
	}

	/**
	 * Getter for the steering motor.
	 * 
	 * @return The steering motor.
	 */
	public TalonFX getSteeringMotor()
	{
		return this.steeringMotor;
	}

	/**
	 * Getter for the can coder.
	 * 
	 * @return The can coder
	 */
	public CANcoder getCanCoder()
	{
		return this.canCoder;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving Motor ID = ?, Cancoder ID = ?]"
		return "SwerveModule[Steering Motor ID = " + this.steeringMotor.getDeviceID() + ", Driving Motor ID = " + this.drivingMotor.getDeviceID() + ", Cancoder ID = " + this.canCoder.getDeviceID() + "]";
	}
}
