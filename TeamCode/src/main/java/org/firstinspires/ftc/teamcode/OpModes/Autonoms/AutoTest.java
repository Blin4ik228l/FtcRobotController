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

    public double returnDistance(double VelMax, double assel){
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

            double linearVel; // Линейная скорость робота

            Vector2 errorPos = new Vector2();

            // Находим ошибку положения
            errorPos.x = args.position.getX() - robot.odometry.getGlobalPosition().getX();
            errorPos.y = args.position.getY() - robot.odometry.getGlobalPosition().getY();
            double errorHeading = args.position.getHeading() - robot.odometry.getGlobalPosition().getHeading();

            // Направление движения
            Vector2 targetVel = new Vector2(errorPos);
            targetVel.normalize();
            targetVel.rotate(-robot.odometry.getGlobalPosition().getHeading()); // Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!

            // Выбираем скорости в зависимости от величины ошибки
            if (errorPos.length() > returnDistance(args.max_linear_speed, robot.MAX_LINEAR_ACCEL)) {
                linearVel = args.max_linear_speed; //Максимально допустимая скорость с args
            } else {
                linearVel = robot.MIN_LINEAR_SPEED;
            }

            if (linearVel < robot.MIN_LINEAR_SPEED) linearVel = robot.MIN_LINEAR_SPEED;// Ограничиваем скорость снизу

            if (errorPos.length() < 2){
                errorPosDone = true;
                linearVel = 0;
            }

            if(Math.abs(errorHeading) < Math.toRadians(2)){
                errorHeadingDone = true;
            }

            targetVel.multyplie(linearVel);

            // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
            double speedPIDX = robot.pidLinearX.calculate(targetVel.x, robot.odometry.getVelocity().x);
            double speedPIDY = robot.pidLinearY.calculate(targetVel.y, robot.odometry.getVelocity().y);
            double angularPID = robot.pidAngular.calculate(args.position.heading, robot.odometry.getGlobalPosition().getHeading());

            if(errorPosDone && errorHeadingDone){
                robot.drivetrain.offMotors();
                return;
            }else{
                robot.drivetrain.setXYHeadVel(speedPIDX, speedPIDY, angularPID);
            }

            robot.messageTelemetry.telemetry.addLine();
            robot.messageTelemetry.addData("Оставшийся угол", errorHeading);
            robot.messageTelemetry.addData("Оставшийся расстояние", errorPos.length());
            robot.messageTelemetry.telemetry.addLine();
            robot.messageTelemetry.showMotorsDriveTrainVoltage();

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
                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1));
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive2));
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive3));
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive4));

            }
        }
}
