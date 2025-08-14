package org.firstinspires.ftc.teamcode.Modules.Handlers;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MathUtils.PID;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MecanumDriveTrain;

public class DriveHandler extends Handler {
    public DriveHandler(MecanumDriveTrain driveTrain, Telemetry telemetry) {
        super(telemetry);
        this.driveTrain = driveTrain;

    }
    public void setArgs(Args.DriveArgs driveArgs){
        this.driveArgs = driveArgs;
    }
    public MecanumDriveTrain driveTrain;
    public Args.DriveArgs driveArgs;
    double speedPIDX;
    double speedPIDY;
    double angularPID;
    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    double I = 0.75;
    double kPang = 1;
    double kPlin = 0.024;
    public final PID pidLinearX = new PID(kPlin,0.000000080,0.000, -I,I);
    public final PID pidLinearY = new PID(kPlin,0.000000080,0.000, -I,I);
    public final PID pidAngular = new PID(kPang,0.0000000,0.00, -I,I);
    Vector2 errorPos;
    double linearVel;// Линейная скорость робота
    double errorHeading;
    Vector2 targetVel;
    private double returnDistance(double VelMax, double accel ){
        return Math.pow(VelMax, 2) / (2 * accel);
    }
    @Override
    public void execute(){
        boolean errorPosDone = false;
        boolean errorHeadingDone = false;

        errorPos = new Vector2();

        // Находим ошибку положения
        errorPos.x = driveArgs.position.getX() - driveTrain.odometry.getGlobalPosition().getX();
        errorPos.y = driveArgs.position.getY() - driveTrain.odometry.getGlobalPosition().getY();
        errorHeading = driveArgs.position.getHeading() - driveTrain.odometry.getGlobalPosition().getHeading();

        // Направление движения
        targetVel = new Vector2(errorPos);
        targetVel.normalize();
        targetVel.rotate(-driveTrain.odometry.getGlobalPosition().getHeading()); // Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!

        // Выбираем скорости в зависимости от величины ошибки
        linearVel = errorPos.length() > returnDistance(driveArgs.speed, MAX_LINEAR_ACCEL) ? driveArgs.speed : MIN_LINEAR_SPEED;

//            if(linearVel <= MID_LINEAR_SPEED){
//                midSpeed -= 1;
//            }

        if(linearVel < MIN_LINEAR_SPEED) linearVel = MIN_LINEAR_SPEED;// Ограничиваем скорость снизу

        if(Math.abs(errorHeading) < Math.toRadians(8)){
            kPang += 0.45;
        } else if (Math.abs(errorHeading) >= Math.toRadians(8) && Math.abs(errorHeading) < Math.toRadians(15)) {
            kPang = 0.50;
        } else{
            kPang = 1;
        }

        if (errorPos.length() < 0.5){
            errorPosDone = true;
            linearVel = 0;
        }

        if(Math.abs(errorHeading) < Math.toRadians(0.5) ){
            errorHeadingDone = true;
            kPang = 0.0;
        }

        pidAngular.setPID(kPang, 0,0);

        targetVel.multyplie(linearVel);

        // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
        speedPIDX = pidLinearX.calculate(targetVel.x, driveTrain.odometry.getVelocity().x);
        speedPIDY = pidLinearY.calculate(targetVel.y, driveTrain.odometry.getVelocity().y);
        angularPID = pidAngular.calculate(driveArgs.position.getHeading(), driveTrain.odometry.getGlobalPosition().getHeading());

        if(errorPosDone && errorHeadingDone){
            speedPIDY = 0;
            speedPIDX = 0;
            angularPID = 0;

            isDone = true;
        }

        driveTrain.setPower(speedPIDY, speedPIDX, angularPID);

        showData();
    }

    @Override
    public void showData() {
        telemetry.addLine("driveToPosition")
                .addData("\nkPang", kPang)
                .addData("\nВектор", targetVel.length())
                .addData("\nВектор X", targetVel.x)
                .addData("\nВектор Y", targetVel.y)
                .addData("\nОставшийся угол", errorHeading * 57.29)
                .addData("\nangularPID", angularPID)
                .addData("\nspeedPIDX", speedPIDX)
                .addData("\nspeedPIDY", speedPIDY)
                .addData("\nОставшийся расстояние", errorPos.length())
                .addData("\nОставшийся X", errorPos.x)
                .addData("\nОставшийся Y", errorPos.y)
                .addData("\nisDone", isDone);


        driveTrain.odometry.getEncPos();
        driveTrain.odometry.getRobotPos();

        telemetry.update();
    }
}
