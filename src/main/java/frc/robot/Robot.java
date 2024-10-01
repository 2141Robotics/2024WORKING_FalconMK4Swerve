package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.pathing.ConstraintsPair;
import com.mineinjava.quail.pathing.Path;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.Util;
import com.mineinjava.quail.util.geometry.AccelerationLimitedDouble;
import com.mineinjava.quail.util.geometry.AccelerationLimitedVector;
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;

import frc.robot.math.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.driveCommand;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.components.QuailSwerveModule;




/**
 * The main robot class where everything is run.
 * 
 * @author 2141 Spartonics
 */
public class Robot extends TimedRobot
{

	public QuailSwerveDrive driveTrain;

	public SwerveOdometry odometry;

	public PathFollower pathFollower;

	public MiniPID pidcontroller;

	AHRS GYRO = new AHRS();

	/** Slot 0 controller. */
	private static final XboxController PRIMARY_CONTROLLER = new XboxController(0);



	@Override
	public void robotInit()
	{
		List<QuailSwerveModule> modules = new ArrayList<QuailSwerveModule>();

		modules.add(new QuailSwerveModule(new Vec2d(-12, 12), 2, 1, 11, -0.76220703125));
		modules.add(new QuailSwerveModule(new Vec2d(-12, -12), 4, 3, 12, -0.64501953125));
		modules.add(new QuailSwerveModule(new Vec2d(12,-12), 6, 5, 13, -0.758056640625));
		modules.add(new QuailSwerveModule(new Vec2d(12,12), 8, 7, 14, -0.824951171875));

		driveTrain = new QuailSwerveDrive(GYRO, modules);

		odometry = new SwerveOdometry(driveTrain);

	}
    public AccelerationLimitedVector a_leftStickVector = new AccelerationLimitedVector(0.1);
    public AccelerationLimitedVector a_rightStickVector = new AccelerationLimitedVector(0.1);

    public AccelerationLimitedVector a_driveVector = new AccelerationLimitedVector(0.003);

    public AccelerationLimitedDouble a_rtrigger = new AccelerationLimitedDouble(0.1);


	@Override
	public void teleopInit(){
		
		double leftX = PRIMARY_CONTROLLER.getLeftX();
		double leftY = - PRIMARY_CONTROLLER.getLeftY(); /// Y UP is negative
		double rightY = -PRIMARY_CONTROLLER.getRightY();
		double rightX = -PRIMARY_CONTROLLER.getRightX();

		double rightTrigger = PRIMARY_CONTROLLER.getRightTriggerAxis();
		
		Vec2d leftStickVector = new Vec2d(leftX, leftY);
        Vec2d rightStickVector = new Vec2d(rightX, rightY);

        Vec2d lstick = a_leftStickVector.update(leftStickVector);
        Vec2d rstick = a_rightStickVector.update(rightStickVector);
        double a_rtriggerValue = a_rtrigger.update(rightTrigger);

		double speedScale = 0.08 + (0.92 * rightTrigger);


		if (Math.abs(rstick.x) < 0.1){
			rightX = Double.MIN_NORMAL;
		}

		if (lstick.getLength() < Constants.deadZonePercent) {
			leftStickVector = new Vec2d(0,0);
		}
        if (rstick.getLength() < Constants.deadZonePercent) {
			rightStickVector = new Vec2d(0,0);
		}
        Vec2d driveVector = leftStickVector.normalize().scale(speedScale);
        Vec2d newDriveVector = a_driveVector.update(driveVector);

        System.out.println(newDriveVector.getLength());


		if ((Math.abs(rstick.x) < 0.1) && (lstick.getLength() < 0.05)){
			driveTrain.stop();
			if (PRIMARY_CONTROLLER.getAButton()) {
				driveTrain.XLockModules();
			}
			driveTrain.brakeOn();
		}
		else {
			
			RobotMovement movement = new RobotMovement(rightStickVector.x / 35, newDriveVector);
			driveTrain.move(movement, this.GYRO.getAngle() * Constants.DEG_TO_RAD);
		}
		if(PRIMARY_CONTROLLER.getYButton()){
			GYRO.reset();
		}
		System.out.println(PRIMARY_CONTROLLER.getLeftX());
	}
	@Override
	public void disabledExit()
	{
		driveTrain.softResetMotors();
	}

	@Override
	public void teleopPeriodic()
	{


	}

	@Override
	public void autonomousInit() {

		ArrayList<Pose2d> points = new ArrayList<Pose2d>();

		points.add(new Pose2d(this.odometry.x,this.odometry.y, 0));
		points.add(new Pose2d(0,144,0));
		points.add(new Pose2d(24,144,0));
		points.add(new Pose2d(24,120,0));
		points.add(new Pose2d(0,120,0));

		Path curPath = new Path(points);
		ConstraintsPair TranslationPair = new ConstraintsPair(10, 10);
		ConstraintsPair RotationPair = new ConstraintsPair(4, 10);
		this.pidcontroller = new MiniPID(0.002,0.000,0, 2);
		this.pidcontroller.setDeadband(0.001/12);
		
		this.pathFollower = new PathFollower(odometry, curPath, TranslationPair, RotationPair, pidcontroller, 3, 0, 1, 2);

		this.odometry.setPose(new Pose2d(0,0,0));
		this.GYRO.reset();
		this.driveTrain.resetMotors();
		this.driveTrain.softResetMotors();
		
	}

	@Override
	public void robotPeriodic(){
		CommandScheduler.getInstance().run();
		

		RobotMovement deltaPos = odometry.calculateFastOdometry(driveTrain.getModuleSpeeds());

		SmartDashboard.putNumber("speed", deltaPos.translation.getLength());

		odometry.updateDeltaPoseEstimate(deltaPos.translation.scale(0.02).rotate(-GYRO.getAngle(), true));
		
		odometry.setAngle(-GYRO.getAngle() * Constants.DEG_TO_RAD);

		SmartDashboard.putNumber("heading", this.GYRO.getAngle());
		SmartDashboard.putNumber("xpos", odometry.x);
		SmartDashboard.putNumber("ypos", odometry.y);


		if (PRIMARY_CONTROLLER.getBButtonPressed()){
			odometry.setPose(new Pose2d(0,0,0));
		}
		

		double[] pos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
		SmartDashboard.putNumberArray("Limelight Pos", pos);
		SmartDashboard.putNumber("LX", -pos[1] * Constants.METERS_TO_INCHES);
		SmartDashboard.putNumber("LY", pos[0] * Constants.METERS_TO_INCHES);

		if(pos[0] != 0 && pos[1] != 0){
			odometry.setPose(new Pose2d(-pos[1]* Constants.METERS_TO_INCHES, pos[0]* Constants.METERS_TO_INCHES, -this.GYRO.getAngle() * Constants.DEG_TO_RAD));
		}

	}

	@Override
	public void autonomousPeriodic() {
		RobotMovement nextMovement = pathFollower.calculateNextDriveMovement();		
		Vec2d newTranslation = (new Vec2d(nextMovement.translation.x/200, nextMovement.translation.y/200));
		double rotation = nextMovement.rotation;


		SmartDashboard.putNumber("deltaAngle", Constants.RAD_TO_DEG * Util.deltaAngle(GYRO.getAngle() * Constants.DEG_TO_RAD, 0));
		SmartDashboard.putNumber("PID output", rotation);

		SmartDashboard.putNumber("autoX", nextMovement.translation.x);
		SmartDashboard.putNumber("autoY", nextMovement.translation.y);

		driveTrain.move(new RobotMovement(rotation/20, newTranslation), GYRO.getAngle() * Constants.DEG_TO_RAD);
		SmartDashboard.putNumber("x", newTranslation.x);
		SmartDashboard.putNumber("y", newTranslation.y);

		if(this.pathFollower.isFinished()){
			driveTrain.stop();
		}
	}

	@Override
	public void autonomousExit() {

	}

}
