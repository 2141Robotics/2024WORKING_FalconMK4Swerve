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
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;

import frc.robot.math.Constants;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.components.QuailSwerveModule;


/**
 * The main robot class where everything is run.
 * 
 * @author 2141 Spartonics
 */
public class Robot extends TimedRobot
{

	public QuailSwerveDrive drivetrain;

	public SwerveOdometry odometry;

	public PathFollower pathFollower;

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

		drivetrain = new QuailSwerveDrive(GYRO, modules);

		odometry = new SwerveOdometry(drivetrain);

	}

	@Override
	public void teleopInit(){
		System.out.print("init");
		robotInit();
	}
	@Override
	public void disabledExit()
	{

	}

	@Override
	public void teleopPeriodic()
	{
		double leftX = PRIMARY_CONTROLLER.getLeftX();
		double leftY = - PRIMARY_CONTROLLER.getLeftY(); /// Y UP is negative
		double rightY = -PRIMARY_CONTROLLER.getRightY();
		double rightX = PRIMARY_CONTROLLER.getRightX();

		double rightTrigger = PRIMARY_CONTROLLER.getRightTriggerAxis();
		
		Vec2d movementVector = new Vec2d(leftX, leftY);

		double speedScale = 0.08 + (0.92 * rightTrigger);

		if (Math.abs(rightX) < 0.1){
			rightX = 0.0001;
		}

		if (movementVector.getLength() < 0.1) {
			movementVector = new Vec2d(0,0);
		}

		if ((Math.abs(rightX) < 0.1) && (movementVector.getLength() < 0.05)){
			drivetrain.stop();
			if (PRIMARY_CONTROLLER.getAButton()) {
				drivetrain.XLockModules();
			}
			drivetrain.brakeOn();
		}
		else {
			rightX /= 35;
			RobotMovement movement = new RobotMovement(rightX, movementVector.normalize().scale(speedScale));
			drivetrain.move(movement, this.GYRO.getAngle() * Constants.DEG_TO_RAD);
		}
		if(PRIMARY_CONTROLLER.getYButton()){
			GYRO.reset();
		}

	}

	@Override
	public void autonomousInit() {
		ArrayList<Pose2d> points = new ArrayList<Pose2d>();

		points.add(new Pose2d(0,12,0));
		points.add(new Pose2d(0,-12,0));

		Path curPath = new Path(points);
		ConstraintsPair CP = new ConstraintsPair(10, 100);
		MiniPID pidcontroller = new MiniPID(0,0,0);
		this.pathFollower = new PathFollower(odometry, curPath, CP, CP, pidcontroller, 0.5, 0, 1, 5);

		this.odometry.setPose(new Pose2d(0,0,0));
		this.GYRO.reset();
		System.out.println("auton init");
		
	}

	@Override
	public void robotPeriodic(){
		CommandScheduler.getInstance().run();

		RobotMovement deltaPos = odometry.calculateFastOdometry(drivetrain.getModuleSpeeds());

		odometry.updateDeltaPoseEstimate(deltaPos.translation.scale(0.02).rotate(-GYRO.getAngle(), true));
		
		odometry.setAngle(GYRO.getAngle() * Constants.DEG_TO_RAD);

		SmartDashboard.putNumber("heading", this.GYRO.getAngle());
		SmartDashboard.putNumber("xpos", odometry.x);
		SmartDashboard.putNumber("ypos", odometry.y);

		if (PRIMARY_CONTROLLER.getBButtonPressed()){
			odometry.setPose(new Pose2d(0,0,0));
		}
	}

	

	
	@Override
	public void autonomousPeriodic() {
		RobotMovement nextMovement = pathFollower.calculateNextDriveMovement();		
		Vec2d newTranslation = (new Vec2d(nextMovement.translation.x/200, nextMovement.translation.y/200));

		SmartDashboard.putNumber("autoX", nextMovement.translation.x);
		SmartDashboard.putNumber("autoY", nextMovement.translation.y);

		drivetrain.move(new RobotMovement(newTranslation), GYRO.getAngle());
		SmartDashboard.putNumber("x", newTranslation.x);
		SmartDashboard.putNumber("y", newTranslation.y);

		System.out.println("auto");
	}

	@Override
	public void autonomousExit() {

	}

}