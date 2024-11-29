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
    Position posForDrive1 = new Position(100, 0 , Math.toRadians(75));
    Position posForDrive2 = new Position( 200, 100, Math.toRadians(90));
    Position posForDrive3 = new Position( 0, 100, Math.toRadians(70));
    Position posForDrive4 = new Position( 0, 0,0 );

    boolean off;
    public double returnDistance(double VelMax, double assel ){
        return Math.pow(VelMax, 2) / (2 * assel);
    }

    public double returnSpeed(double target, double assel){
        return Math.sqrt(2 * target * assel);
    }

    public void driveMethod(StandartArgs.driveStandartArgs args){
        off = false;
        while(!isStopRequested()){
            boolean errorPosDone = false;
            boolean errorHeadingDone = false;

            double accel = robot.MAX_LINEAR_ACCEL;

            double linearVelX; // Линейная скорость робота X
            double linearVelY;// Линейная скорость робота Y
            double angularVel; // Угловая скорость робота

            // Находим ошибку положения
            double errorX = args.position.x - robot.odometry.getGlobalPosition().x;
            double errorY = args.position.y - robot.odometry.getGlobalPosition().y;
            double errorHeading = args.position.heading - robot.odometry.getGlobalPosition().getHeading();

            Vector2 errorPos = new Vector2(errorX,errorY);
            // Направление движения
            Vector2 targetVel = new Vector2(errorPos);
            targetVel.normalize();
            targetVel.rotate(robot.odometry.getGlobalPosition().getHeading()); // Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!

            // Выбираем скорости в зависимости от величины ошибки
            if (Math.abs(errorPos.x) > returnDistance(args.max_linear_speed, accel)) {
                linearVelX = args.max_linear_speed; //Максимально допустимая скорость с args
            } else {
//                linearVelX = returnSpeed(Math.abs(errorPos.x), accel);
                linearVelX = robot.MIN_LINEAR_SPEED;
            }


            if (Math.abs(errorPos.y) > returnDistance(args.max_linear_speed, accel)) {
                linearVelY = args.max_linear_speed; //Максимально допустимая скорость с args
            } else {
//                linearVelY = returnSpeed(Math.abs(errorPos.y), accel);
                linearVelY = robot.MIN_LINEAR_SPEED;
            }

            if (linearVelX < robot.MIN_LINEAR_SPEED) linearVelX = robot.MIN_LINEAR_SPEED;;// Ограничиваем скорость снизу

            if (linearVelY < robot.MIN_LINEAR_SPEED) linearVelY = robot.MIN_LINEAR_SPEED; // Ограничиваем скорость снизу

           if(errorPos.length() < 2){
                errorPosDone = true;
                linearVelY = 0;
                linearVelX = 0;
            }

            if(Math.abs(errorHeading)< Math.toRadians(1)){
                errorHeadingDone = true;
                args.position.heading = 0;
            }

            // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
            double speedPIDX = robot.pidLinearX.calculate(linearVelX, robot.odometry.getVelocity().x);
            double speedPIDY = robot.pidLinearY.calculate(linearVelY, robot.odometry.getVelocity().y);
            double angularPID = robot.pidAngular.calculate(args.position.heading,robot.odometry.getGlobalPosition().getHeading() );

            double targetVelX = targetVel.x * speedPIDX;
            double targetVelY = targetVel.y * speedPIDY;

            if(errorPosDone && errorHeadingDone){
                robot.drivetrain.offMotors();
                return;
            }else{
                robot.drivetrain.setXYHeadVel(targetVelX, targetVelY, angularPID);
            }

            robot.messageTelemetry.addData("targetVel",targetVel.length());
            robot.messageTelemetry.addData("targetVelX",targetVel.x);
            robot.messageTelemetry.addData("targetVelY",targetVel.y);
            robot.messageTelemetry.telemetry.addLine();
            robot.messageTelemetry.addData("Угол", robot.odometry.getGlobalPosition().getHeading());
            robot.messageTelemetry.addData("Оставшийся угол", errorHeading);
            robot.messageTelemetry.telemetry.update();
        }
    }

        @Override
        public void runOpMode() throws InterruptedException {
            robot = new Robot(RobotMode.AUTO,RobotAlliance.RED, this);
            robot.init();
            waitForStart();
            while (opModeIsActive()) {
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1, 200));
                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1, 100, 6));
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive2));
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive3));
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive4));

            }
        }
}
