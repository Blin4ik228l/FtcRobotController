package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

@Autonomous(name = "Test", group = "Test")
public class AutoTest extends LinearOpMode {
    Robot robot;
    Position posForDrive1 = new Position(100, 0 , 0);
    Position posForDrive2 = new Position( 0, 0, Math.toRadians(90));
    Position posForDrive3 = new Position( 0, 100, 0);
    Position posForDrive4 = new Position( 0, -100, Math.toRadians(70));

    public double returnDistance(double VelMax, double assel ){
        return Math.pow(VelMax, 2) / (2 * assel);
    }

    public double returnSpeed(double target, double assel){
        return Math.sqrt(2 * target * assel);
    }

    public void driveMethod(StandartArgs.driveStandartArgs args){
        double accel = robot.MAX_LINEAR_ACCEL;

        double linearVel; // Линейная скорость робота
        double angularVel; // Угловая скорость робота

        // Находим ошибку положения
        Vector2 errorPos = args.position.toVector();
        errorPos.sub(robot.odometry.getGlobalPosition().toVector());

        double errorHeading = args.position.heading - robot.odometry.getGlobalPosition().getHeading();

        // Направление движения
        Vector2 targetVel = new Vector2(errorPos);
        targetVel.normalize();
        targetVel.rotate(-robot.odometry.getGlobalPosition().getHeading()); // Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!


        // Выбираем скорости в зависимости от величины ошибки
        if (errorPos.length() > returnDistance(args.max_linear_speed, accel)) {
            linearVel = args.max_linear_speed; //Максимально допустимая скорость с args
        } else {
            linearVel = returnSpeed(errorPos.length(), accel);
        }

        if (linearVel < robot.MIN_LINEAR_SPEED) linearVel = robot.MIN_LINEAR_SPEED; // Ограничиваем скорость снизу

        if(linearVel > args.max_linear_speed) linearVel = args.max_linear_speed;

        if(errorHeading > 0.6) { // 0.6 рад = 35 град
            angularVel = args.max_angular_speed;
        } else {
            angularVel = robot.MIN_ANGULAR_SPEED;
            if(angularVel > args.max_angular_speed) angularVel = args.max_angular_speed;
        }

        // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
        double speedPID = robot.pidLinear.calculate(linearVel, robot.odometry.getSpeed());
        double angularPID = robot.pidAngular.calculate(angularVel, robot.odometry.getAngularVelocity());

        robot.messageTelemetry.addData("targetVelX",targetVel.x );
        robot.messageTelemetry.addData("targetVelY",targetVel.y );
        robot.messageTelemetry.telemetry.addLine();
        robot.messageTelemetry.addData("errorPosX",errorPos.x);
        robot.messageTelemetry.addData("errorPosY",errorPos.y);
        robot.messageTelemetry.telemetry.addLine();
        robot.messageTelemetry.showMotorsDriveTrainVoltage();
//        robot.messageTelemetry.addData("Расстояние до цели", errorPos.length());
//        robot.messageTelemetry.addData("Оставшийся угол", errorHeading);

        robot.messageTelemetry.telemetry.update();

        targetVel.multyplie(speedPID);

        if(errorPos.length() < 2 && Math.abs(errorHeading)< Math.toRadians(3) ){
            robot.drivetrain.offMotors();
        }else{
//            robot.drivetrain.setVelocity(targetVel, angularPID);
        }



    }

        @Override
        public void runOpMode() throws InterruptedException {
            robot = new Robot(RobotMode.AUTO,RobotAlliance.RED, this);
            robot.init();
            waitForStart();
            while (opModeIsActive()) {
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1, 200));
                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1, 0, 0));

            }
        }
}
