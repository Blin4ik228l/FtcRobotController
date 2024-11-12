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
        Position posForDrive1 = new Position(300, 0 , 0);
        Position posForDrive2 = new Position( 0, 100, 0);
        Position posForDrive3 = new Position( 50, 50, 0);

    double maxCurrentSpeed = 0;
    double currentSpeed = 0;
    double pastSpeed = 0;

        public void driveMethod(StandartArgs.driveStandartArgs args){

                double assel;

                if(args.position.x > 0 || args.position.x < 0 && args.position.y == 0){
                    assel = robot.MAX_ACCEL_FRONT;
                } else if (args.position.x == 0 && args.position.y > 0 || args.position.y < 0) {
                    assel = robot.MAX_ACCEL_SIDE;
                }else {
                    assel = Math.sqrt(Math.pow(robot.MAX_ACCEL_FRONT, 2) + Math.pow(robot.MAX_ACCEL_SIDE, 2));
                }

                double Vel;

                double VelMax = args.max_linear_speed;
                double Vel0 = robot.odometry.returnSpeed(args.position, assel);

                if(args.position.toVector().length() > robot.odometry.returnDistance(Vel0, assel)){
                    Vel = Vel0 ;
                }else{
                    Vel = VelMax;
                }

                currentSpeed = Vel;
                double errorHeading = args.position.heading - robot.odometry.getGlobalPosition().getHeading();
                Vector2 target = args.position.toVector();

                target.sub(robot.odometry.getGlobalPosition().toVector());

            double speedPID = robot.pidLinear.calculate(Vel, robot.odometry.getSpeed());
//            double angularPID = robot.pidAngular.calculate(args.max_angular_speed, robot.odometry.getAngularVelocity()); //Всегда положителен

                double headingVel =  errorHeading;

                Vector2 targetVel = new Vector2(target);

                targetVel.normalize();

                targetVel.multyplie(-speedPID);


//            if(Math.abs(targetVel.length()) < 0.10) {
//                targetVel.setVectorLength((targetVel.length() / Math.abs(targetVel.length()) * 0.11));
//            }

            robot.drivetrain.setVelocity(targetVel, headingVel);

//            else if (Math.abs(headingVel) < 0.11) {
//                headingVel = (headingVel/Math.abs(headingVel)) * 0.11;
//            }

            if (target.length() < 2){
                robot.drivetrain.offMotors();
          
            }

            if(maxCurrentSpeed < robot.odometry.getSpeed()){
                maxCurrentSpeed = robot.odometry.getSpeed();
            }

            if(targetVel.length() != 0){
                pastSpeed = robot.odometry.getSpeed();
            }

            robot.messageTelemetry.addData("P", robot.pidLinear.P);
            robot.messageTelemetry.addData("I", robot.pidLinear.I);
            robot.messageTelemetry.addData("D", robot.pidLinear.D);
            robot.messageTelemetry.addData("PID", -speedPID);
            robot.messageTelemetry.addData("Нынешняя скорость", robot.odometry.getSpeed());
            robot.messageTelemetry.addData("Последняя скорость", pastSpeed);
            robot.messageTelemetry.addData("Выбранная скорость", Vel);
            robot.messageTelemetry.addData("Максмимальная из нынешней скорости", maxCurrentSpeed);
//            robot.messageTelemetry.addData("Оставшееся растояние", target.length());
            robot.messageTelemetry.telemetry.addLine();
            robot.messageTelemetry.telemetry.update();
        }

        @Override
        public void runOpMode() throws InterruptedException {
            robot = new Robot(RobotMode.AUTO,RobotAlliance.RED, this);
            robot.init();
            waitForStart();
            while (opModeIsActive()) {
                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1, 100));

//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive2, 30));
            }
        }
}
