package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.PositionRobotController;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;

public class MecanumDrivetrain extends UpdatableModule {
    //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.
    public TeamClass teamClass;
    public DrivetrainMotors motors;
    public OdometryClass odometryClass;
    public CameraClass cameraClass;
    public PositionRobotController positionRobotController;
    public MecanumDrivetrain(TeamClass.Color color, TeamClass.StartPos startPos, OpMode op){
        super(op.telemetry);
        teamClass = new TeamClass(color, startPos, op);

        motors = new DrivetrainMotors(op);

        odometryClass = new OdometryClass(op);
        cameraClass = new CameraClass(op);

        positionRobotController = new PositionRobotController(odometryClass, teamClass, cameraClass, op);

        telemetry.addLine("Drivetrain Inited");
    }

    @Override
    public void update() {
        positionRobotController.update();
    }

    @Override
    public void showData() {
        teamClass.showData();
        cameraClass.showData();
        odometryClass.showData();
        positionRobotController.showData();
        motors.showData();
    }
}
