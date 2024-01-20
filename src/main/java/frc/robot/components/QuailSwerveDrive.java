package frc.robot.components;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.SwerveDrive;
import com.mineinjava.quail.util.geometry.Vec2d;

import frc.robot.math.Constants;

/**
 * Container for the robot's swerve drivetrain. Wraps a gyroscope and swerve modules.
 * 
 * @author 2141 Spartonics
 */
public class QuailSwerveDrive extends SwerveDrive<QuailSwerveModule>
{
	/** A timer used to continually reset the motors. Prevents overshooting the rotation. */
	private static int resetTimer = 0;
	/** The gryoscope used for rotaiton measurements. */
	private final AHRS gyro;
	/** A list of all of the swerve modules on the drivetrain. */
	private final List<QuailSwerveModule> modules;
	
	/**
	 * @param minSpeed minimum movement speed (0 to 1)
	 * @param maxSpeed maximum movement speed (0 to 1)
	 * @param maxRotation maximum rotational speed (0 to 1)
	 * @param gyroscope the swerve drive's gyroscope
	 * @param QuailSwerveModules the swerve drive's wheel modules
	 */
	public QuailSwerveDrive(AHRS gyroscope, List<QuailSwerveModule> modules)
	{
		super(modules);
		this.gyro = gyroscope;
		this.modules = modules;
		for (QuailSwerveModule module : modules) {
			module.init();
		}
	}

	
	/**
	 * Set a timer for reseting the motors.
	 * Used to reset the motors multiple times.
	 */
	public void resetMotors()
	{
		this.modules.forEach(m -> m.reset());

	}

	/**
	 * Reset the gyro to 0°.
	 */
	public void resetGyro()
	{
		this.gyro.reset();
	}
	

	/**
	 * Checks if the robot is resetting or if the gyro is callibrating.
	 * 
	 * @return Whether or not the robot can drive
	 */
	public boolean canDrive()
	{
		return resetTimer <= 0 && !this.gyro.isCalibrating();
	}

	/**
	 * Getter for the gyroscope.
	 * 
	 * @return The gyroscope.
	 */
	public AHRS getGyro()
	{
		return this.gyro;
	}

	/**
	 * Getter for the swerve modules.
	 * 
	 * @return The swerve modules.
	 */
	public List<QuailSwerveModule> getModules()
	{
		return this.modules;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveDrive[Module1 = {}, Module2 = {}, ...]"
		StringBuilder builder = new StringBuilder("SwerveDrive[");

		for(int i = 0; i < this.modules.size(); i++)
		{
			builder.append("Module" + i + " = {" + this.modules.get(i) + "}, ");
		}

		builder.delete(builder.length() - 2, builder.length());
		builder.append("]");
		return builder.toString();
	}

	public double gettingangle(){
		return this.gyro.getAngle() * Constants.DEG_TO_RAD;
	}


	public void stop(){
		for(QuailSwerveModule module : this.modules) {
			module.drivingMotor.set(0);
		}
	}

    /** 
	 * Set modules into coast mode 
	*/
	public void coastOn(){
		for(QuailSwerveModule module : this.modules) {
			module.drivingMotor.setControl(new CoastOut());
		}
	}

	public void brakeOn(){
		for(QuailSwerveModule module : this.modules) {
			module.drivingMotor.setControl(new StaticBrake());
		}
	}

	public ArrayList<Vec2d> getModuleSpeeds() {
        ArrayList<Vec2d> vectors = new ArrayList<Vec2d>();
        for (QuailSwerveModule module : this.swerveModules) {
            vectors.add(((QuailSwerveModule) module).getCurrentMovement());
        }
        return vectors;
    }


    
	/**
	 * Average encoder distance of the drive modules
	 * @return distance in encoder ticks before gear ratio
	 */
	public double averageDist() {
		int totaldist = 0;
		for (int i = 0; i <4; i++) {
			totaldist += this.modules.get(i).drivingMotor.getPosition().refresh().getValue();
		}
		return totaldist/4;
	}

	public void resetDistance(){
		for (QuailSwerveModule module : this.modules) {
			module.drivingMotor.setPosition(0d);
		}
	}
}
