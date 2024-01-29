package frc.robot.math;

/**
 * Container for various static variables.
 * 
 * @author 2141 Spartonics
 */
public class Constants
{
	/** The delay in milliseconds before a report gets sent to DriverStation if an action fails. */
	public static final int MS_DELAY = 30;

	/** Settings are {kF, kP, kI, kD} *///		kV  kP	 kI  kD
	public static final double[] PID_SETTINGS =  { 0.0d, 0.5d, 0.0d, 0.00d };

	/** 2π */
	public static final double TWO_PI = 2d * Math.PI;

	/** π/2 */
	public static final double PI_OVER_TWO = Math.PI / 2d;

	/** Multiply a radian value by this to convert it to degrees. */
	public static final double RAD_TO_DEG = 180d / Math.PI;

	public static final double DEG_TO_RAD = 1/ RAD_TO_DEG;

	/** Angle precision for rotation */
	public static final double ANGLE_PRECISION = Math.PI / 32;

	public static final int INCHES_PER_FOOT = 12;

	public static final double steeringRatio = 12.8d;
	public static final double driveRatio = 6.75d;

	public static final double deadZonePercent = 0.05d;

	public static final double METERS_TO_INCHES = 39.3701;
}
