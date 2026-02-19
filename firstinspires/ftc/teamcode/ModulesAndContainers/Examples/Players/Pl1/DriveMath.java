package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.TaskAndArgs.Args;
import org.firstinspires.ftc.teamcode.ConstansOrMagicNumbers.AnotherConsts;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.NavigationSystem;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.PID;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.PositionController;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class DriveMath implements AnotherConsts {
    private PID pidLinearX;
    private PID pidLinearY;
    private PID pidAngular;
    private Vector2 voltageVector;
    private double headVoltage;
    private Vector2 levelVector;
    private double levelHeadVoltage;

    public DriveMath(){
        pidLinearX = new PID(3.3, 0,0.000, -1,1);
        pidLinearY = new PID(3.3,0,0.000, -1,1);
        pidAngular = new PID(2.5,0.000,0.00, -1,1);
        voltageVector = new Vector2();
    }

    public Vector2 moveHeadless(double cosA, double sinA) {//FieldCentric
        double heading = -0;// Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!

        double cosB = Math.cos(heading);//  +- угол робота относитеольно поля
        double sinB = Math.sin(heading);

        double globalX = cosA * cosB - sinA * sinB;
        double globalY = sinA * cosB + cosA * sinB;

        voltageVector.x = globalX;
        voltageVector.y = globalY;

        return voltageVector;
    }
    private double returnDistance(double VelMax, double accel ){
        return Math.pow(VelMax, 2) / (2 * accel);
    }

    public Position2D calculateVol(NavigationSystem navigationSystem) {
        boolean isYDone = false;
        boolean isXDone = false;
        boolean isTurnDone = false;

        Position2D targetPos = positionController;
        Position2D currentPos = ;
        // Находим ошибку положения
        Position2D deltaPos = targetPos.minus(currentPos);


        // Направление движения
        Vector2 deltaVector = deltaPos.toVector();
        double errorHeading = deltaPos.getHeading();//Turn

        // Выбираем скорости в зависимости от величины ошибки
        double linearVel = deltaVector.length() > returnDistance(drivetrain.positionRobotController.driveArgs.speed, MAX_LINEAR_ACCEL) ? drivetrain.positionRobotController.driveArgs.speed : MIN_LINEAR_SPEED;// Линейная скорость робота

        deltaVector.normalize();

        Vector2 deltaSpeed = new Vector2(deltaVector.x * linearVel - drivetrain.positionRobotController.getOdometryClass().getRobotCurVelocity().x, deltaVector.y * linearVel - drivetrain.positionRobotController.getOdometryClass().getRobotCurVelocity().y);

//                    cosA = pidLinearX.calculate(deltaSpeed.x);
//                    sinA = pidLinearY.calculate(deltaSpeed.y);

        cosA = Math.signum(deltaVector.x) * (linearVel / MAX_LINEAR_SPEED);
        sinA = Math.signum(deltaVector.y) * (linearVel / MAX_LINEAR_SPEED);

        if (drivetrain.positionRobotController.needVyr) {
            errorHeading = drivetrain.positionRobotController.getDeltaAngle();
            joystickActivityClass.buttonY = true;
        }
        turn = errorHeading / Math.PI;

        if (Math.abs(errorHeading) < Math.toRadians(25)) {
            turn = Math.signum(turn) * 0.15;
        }


        deltaVector.scale();

        if (Math.abs(deltaVector.x) < 3) {
            cosA = 0;
            isXDone = true;
        }

        if (Math.abs(deltaVector.y) < 3) {
            sinA = 0;
            isYDone = true;
        }

        if (Math.abs(errorHeading) < Math.toRadians(2)) {
            turn = 0;
            isTurnDone = true;
        }

        if (isYDone && isXDone && isTurnDone) {
            drivetrain.positionRobotController.generalState = PositionController.GeneralState.DONE;
        }

        return null;
    }
}
