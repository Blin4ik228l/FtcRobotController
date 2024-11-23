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
        Position posForDrive3 = new Position( 0, 100, Math.toRadians(50));
        Position posForDrive4 = new Position( 0, -100, Math.toRadians(70));
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

            double errorHeading = args.position.heading + robot.odometry.getGlobalPosition().getHeading();

            Vector2 target = args.position.toVector();

            target.sub(robot.odometry.getGlobalPosition().toVector());

            if (target.length() > returnDistance(args.max_linear_speed, assel)) {
                Vel = args.max_linear_speed;//Максимально допустимая скорость с args
            } else if (target.length() < (returnDistance(args.max_linear_speed, assel) / 3.5)) {
                Vel = 25;
            } else {
                Vel = returnSpeed(target.length(), assel);
            }

            if(errorHeading > Math.toRadians(40)){
                VelRound = args.max_angular_speed;
            }
            else {
                VelRound = args.max_angular_speed/2.5;
            }

            double speedPID = robot.pidLinear.calculate(Vel, robot.odometry.getSpeed());
            double angularPID = robot.pidAngular.calculate(VelRound, Math.abs(robot.odometry.getAngularVelocity()));

            double headingVel;

            if(errorHeading != 0){
                headingVel = -angularPID * errorHeading/Math.abs(errorHeading);
                if (Math.abs(headingVel) < 0.1) {
                    headingVel = -0.12 * errorHeading/Math.abs(errorHeading);
                }
            }else {
                headingVel = 0;
            }

            Vector2 targetVel = new Vector2(target);

            targetVel.normalize();

            if(Math.abs(speedPID) < 0.1){
                speedPID = 0.12;
            }
            targetVel.multyplie(-speedPID);


            robot.messageTelemetry.addData("Расстояние по прямой", target.length());
            robot.messageTelemetry.addData("Оставшийся угол", errorHeading);
            robot.messageTelemetry.telemetry.addLine();
            robot.messageTelemetry.addData("Напряжение после ПИД по прямой", targetVel.length());
            robot.messageTelemetry.addData("Напряжение после ПИД для угла", headingVel);
            robot.messageTelemetry.telemetry.update();

            if(target.length() < 2 && Math.abs(errorHeading)< Math.toRadians(3) ){
                robot.drivetrain.offMotors();
                return;
            }else{
//                if(target.length() > 2){
//                    robot.drivetrain.setVelocity(targetVel, 0);
//                    return;
//                }
//                if(Math.abs(errorHeading)> Math.toRadians(3)){
//                    robot.drivetrain.setVelocity(new Vector2(0,0), headingVel);
//                    return;
//                }
                robot.drivetrain.setVelocity(targetVel, headingVel);
            }
        }

        @Override
        public void runOpMode() throws InterruptedException {
            robot = new Robot(RobotMode.AUTO,RobotAlliance.RED, this);
            robot.init();
            waitForStart();
            while (opModeIsActive()) {
//                driveMethod(new StandartArgs.driveStandartArgs(posForDrive1, 200));
                driveMethod(new StandartArgs.driveStandartArgs(posForDrive3, 150, 3));

            }
        }
}
