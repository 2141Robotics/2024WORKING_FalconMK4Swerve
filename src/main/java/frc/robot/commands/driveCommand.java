package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.math.Constants;

public class driveCommand extends Command{
    public XboxController PRIMARY_CONTROLLER;
    public AHRS gyro;
    public QuailSwerveDrive driveTrain;
    public driveCommand(XboxController controller1, AHRS gyro1, QuailSwerveDrive driveTrain1){
        super();
        PRIMARY_CONTROLLER = controller1;
        gyro = gyro1;
        driveTrain = driveTrain1;
    }

    @Override
    public void initialize() {
        super.initialize();
        driveTrain.softResetMotors();
        gyro.reset();
        driveTrain.stop();
        System.out.println("drive commmand start");
    }
    @Override
    public void execute() {
        super.execute();
        double leftX = PRIMARY_CONTROLLER.getLeftX();
		double leftY = - PRIMARY_CONTROLLER.getLeftY(); /// Y UP is negative
		double rightY = -PRIMARY_CONTROLLER.getRightY();
		double rightX = PRIMARY_CONTROLLER.getRightX();

		double rightTrigger = PRIMARY_CONTROLLER.getRightTriggerAxis();
		
		Vec2d leftStickVector = new Vec2d(leftX, leftY);
        Vec2d rightStickVector = new Vec2d(rightX, rightY);

        
		double speedScale = 0.08 + (0.92 * rightTrigger);

		if (Math.abs(rightX) < 0.1){
			rightX = 0.0001;
		}

		if (leftStickVector.getLength() < Constants.deadZonePercent) {
			leftStickVector = new Vec2d(0,0);
		}
        if (rightStickVector.getLength() < Constants.deadZonePercent) {
			rightStickVector = new Vec2d(0,0);
		}

		if ((Math.abs(rightX) < 0.1) && (leftStickVector.getLength() < 0.05)){
			driveTrain.stop();
			if (PRIMARY_CONTROLLER.getAButton()) {
				driveTrain.XLockModules();
			}
			driveTrain.brakeOn();
		}
		else {
			rightX /= 35;
			RobotMovement movement = new RobotMovement(-rightX, leftStickVector.normalize().scale(speedScale));
			driveTrain.move(movement, this.gyro.getAngle() * Constants.DEG_TO_RAD);
		}
		if(PRIMARY_CONTROLLER.getYButton()){
			gyro.reset();
		}
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("drive command end");
        
        super.end(interrupted);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
