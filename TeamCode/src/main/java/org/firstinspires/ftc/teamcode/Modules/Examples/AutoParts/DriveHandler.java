package org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.PID;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;

public class DriveHandler extends ExecutableModule {
    public DriveHandler(MecanumDrivetrain driveTrain, OpMode op) {
        super(op.telemetry);
        this.driveTrain = driveTrain;

        pidLinearX = new PID(0.015,0.000000080,0.000, -1,1);
        pidLinearY = new PID(0.015,0.000000080,0.000, -1,1);
        pidAngular = new PID(2.5,0.0000000,0.00, -1,1);
    }
    public void setArgs(Args.DriveArgs driveArgs){
        this.driveArgs = driveArgs;
    }
    public MecanumDrivetrain driveTrain;
    public Args.DriveArgs driveArgs;

    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public PID pidLinearX;
    public PID pidLinearY;
    public PID pidAngular;
    public boolean isDone;

    public double speedPIDX;
    public double speedPIDY;
    public double angularPID;

    Position2D deltaPos = new Position2D();
    Vector2 deltaSpeed = new Vector2();

    public double linearVel;
    private double returnDistance(double VelMax, double accel ){
        return Math.pow(VelMax, 2) / (2 * accel);
    }
    @Override
    public void execute(){
        boolean errorPosDone = false;
        boolean errorHeadingDone = false;

        deltaPos = new Position2D(
                driveArgs.position2D.getX() - driveTrain.odometryClass.getEncGlobalPosition2D().getX(),
                driveArgs.position2D.getY() - driveTrain.odometryClass.getEncGlobalPosition2D().getY(),
                0);

        // Находим ошибку положения
        // Направление движения
        Vector2 deltaVector = deltaPos.toVector().rotateToGlobal(-(Math.toRadians(-90) + driveTrain.odometryClass.getEncGlobalPosition2D().getHeading()));// Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!
        double errorHeading = deltaPos.getHeading();//Turn

        deltaVector.normalize();

        // Выбираем скорости в зависимости от величины ошибки
        linearVel = deltaVector.length() > returnDistance(driveArgs.speed, MAX_LINEAR_ACCEL) ? driveArgs.speed : MIN_LINEAR_SPEED;// Линейная скорость робота

        deltaSpeed = new Vector2(deltaVector.x * linearVel, deltaVector.y * linearVel );

        // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
        speedPIDX = pidLinearX.calculate(deltaSpeed.x, driveTrain.odometryClass.getRobotCurVelocity().x);
        speedPIDY = pidLinearY.calculate(deltaSpeed.y, driveTrain.odometryClass.getRobotCurVelocity().y);
        angularPID = pidAngular.calculate(errorHeading);

        if (deltaVector.length() < 0.5){
            errorPosDone = true;
            speedPIDX = 0.0;
            speedPIDY = 0.0;
        }

        if(Math.abs(errorHeading) < Math.toRadians(1) ){
            errorHeadingDone = true;
            angularPID = 0.0;
        }

        if(errorPosDone && errorHeadingDone){
            isDone = true;
            pidLinearX = new PID(0.010,0.000000080,0.000, -1,1);
            pidLinearY = new PID(0.010,0.000000080,0.000, -1,1);
            pidAngular = new PID(2.5,0.0000000,0.00, -1,1);
        }

        driveTrain.motors.setPower(speedPIDX, speedPIDY, angularPID);

    }

    @Override
    public void showData() {
        telemetry.addLine("===DriveHandler===");
        telemetry.addData("isDone", isDone);
        telemetry.addData("Vel", linearVel);
        telemetry.addData("DeltaSpeed", "X: %.2f Y: %.2f Len: %.2f", deltaSpeed.x, deltaSpeed.y, deltaSpeed.length());
        telemetry.addData("Speeds", "X: %.2f Y: %.2f H: %.2f", speedPIDX, speedPIDY, angularPID);
        telemetry.addData("Deltas", "X: %.2f Y: %.2f H: %.2f", deltaPos.getX(), deltaPos.getY(), deltaPos.getY());
        telemetry.addData("Drive args", "Targets X: %s Y: %s Head: %.2f Speed: %s", driveArgs.position2D.getX(), driveArgs.position2D.getY(), driveArgs.position2D.getHeading(), driveArgs.speed);
        telemetry.addLine();
    }
}
