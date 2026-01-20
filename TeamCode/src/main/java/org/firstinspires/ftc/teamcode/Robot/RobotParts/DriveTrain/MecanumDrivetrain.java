package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryClass;

public class MecanumDrivetrain extends UpdatableModule {
    //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.
    public DrivetrainMotors motors;
    public PositionRobotController positionRobotController;
    public MecanumDrivetrain(OpMode op){
        super(op);

        motors = new DrivetrainMotors(op);

        positionRobotController = new PositionRobotController(op);

        telemetry.addLine("Drivetrain Inited");
    }

    @Override
    public void setIteration(int iteration) {
        super.setIteration(iteration);
        motors.setIteration(iteration);
        positionRobotController.setIteration(iteration);
    }

    @Override
    public void resetTimer() {
        innerRunTime.reset();

        motors.resetTimer();
        positionRobotController.resetTimer();
    }

    @Override
    public void update() {
        positionRobotController.update();
    }

    @Override
    public void showData() {
        positionRobotController.showData();
        motors.showData();
    }
}
