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
        Position posForDrive2 = new Position( 0, 0, Math.toRadians(90));
        Position posForDrive3 = new Position( 0, 100, 0);

    double maxCurrentSpeed = 0, maxAngleSpeed = 0;
    double currentSpeed = 0;
    double pastSpeed = 0;
    boolean off;

    public double returnDistance(double VelMax, double assel ){
        return Math.pow(VelMax, 2)/ (2* assel);
    }

    public double returnSpeed(double target, double assel){
        return Math.sqrt(2 * target * assel);
    }

        public void driveMethod(StandartArgs.driveStandartArgs args){
            double assel;
            double asselRound = robot.MAX_ACCEL_ROUND;

            if (args.position.x > 0 || args.position.x < 0 && args.position.y == 0) {
                assel = robot.MAX_ACCEL_FRONT;
            } else if (args.position.x == 0 && args.position.y > 0 || args.position.y < 0) {
                assel = robot.MAX_ACCEL_SIDE;
            } else {
                assel = robot.MAX_ACCEL_FRONT;
            }

            double Vel;
            double VelRound;

            double errorHeading = Math.abs(args.position.heading) - Math.abs(robot.odometry.getGlobalPosition().getHeading());

            Vector2 target = args.position.toVector();

            target.sub(robot.odometry.getGlobalPosition().toVector());

            if (target.length() > returnDistance(args.max_linear_speed, assel)) {
                Vel = args.max_linear_speed;//Максимально допустимая скорость с args
            } else if (target.length() < (returnDistance(args.max_linear_speed, assel) / 3.5)) {
                Vel = 25;
            } else {
                Vel = returnSpeed(target.length(), assel);
            }

            if(errorHeading > returnDistance(args.max_angular_speed, asselRound)){
                VelRound = args.max_angular_speed;
            }
            else {
                VelRound = args.max_angular_speed/1.5;
            }

            double speedPID = robot.pidLinear.calculate(Vel, robot.odometry.getSpeed());
            double angularPID = robot.pidAngular.calculate(VelRound, Math.abs(robot.odometry.getAngularVelocity()));

            double headingVel = -angularPID * errorHeading/Math.abs(errorHeading);

            Vector2 targetVel = new Vector2(target);

            targetVel.normalize();

            targetVel.multyplie(-speedPID);


//            if(Math.abs(targetVel.length()) < 0.10) {
//                targetVel.setVectorLength((targetVel.length() / Math.abs(targetVel.length()) * 0.11));
//            }

//            if (Math.abs(headingVel) < 0.11) {
//                headingVel = (headingVel/Math.abs(headingVel)) * 0.11;
//            }
            if (target.length() < 2 && errorHeading < Math.toRadians(5)) {
                robot.drivetrain.offMotors();
                return;
            }

            robot.drivetrain.setVelocity(targetVel, headingVel);

            if (maxCurrentSpeed < robot.odometry.getSpeed()) {
                maxCurrentSpeed = robot.odometry.getSpeed();
            }

            if(maxAngleSpeed < Math.abs(robot.odometry.getAngularVelocity())){
                maxAngleSpeed = Math.abs(robot.odometry.getAngularVelocity());
            }

            if (targetVel.length() != 0) {
                pastSpeed = robot.odometry.getSpeed();
            }

//            robot.messageTelemetry.addData("P", robot.pidLinear.P);
//            robot.messageTelemetry.addData("I", robot.pidLinear.I);
//            robot.messageTelemetry.addData("D", robot.pidLinear.D);
//            robot.messageTelemetry.addData("PID", -speedPID);
            robot.messageTelemetry.addData("PIDANGULAR", angularPID);
            robot.messageTelemetry.addData("Выбранная угловая скорость", VelRound);
            robot.messageTelemetry.addData("Максимальная угловая скорость", maxAngleSpeed);
            robot.messageTelemetry.addData("Угловая скорость", robot.odometry.getAngularVelocity());
            robot.messageTelemetry.addData("Оставшейся угол", errorHeading);
            robot.messageTelemetry.addData("Угол", robot.odometry.getGlobalPosition().heading);
//            robot.messageTelemetry.addData("Последняя скорость", pastSpeed);
//            robot.messageTelemetry.addData("Выбранная скорость", Vel);
//            robot.messageTelemetry.addData("Максмимальная из нынешней скорости", maxCurrentSpeed);
//            robot.messageTelemetry.addData("Нынешняя скорость", robot.odometry.getSpeed());
//            robot.messageTelemetry.showMotorsDriveTrainVoltage();
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
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1, 200));
                driveMethod(new StandartArgs.driveStandartArgs(posForDrive2, 0, Math.toRadians(360)));

            }
        }
}
