package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.TeleSkope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.MessageTelemetry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.PID;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

public class Robot extends RobotCore implements CONSTS{
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Системы робота.
    // Железо хранится уже в самой системе.
    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота
    public final TeleSkope teleSkope;

    public final MessageTelemetry messageTelemetry;
    public final ElapsedTime time = new ElapsedTime();
    public double timeAngle, timeVel, oldTime;
    double maxTime = 0, maxAngleAccel = 0, maxVelAAngle = 0, headingWhenReachMaxVel = 0, headingWhenReachMaxAngle = 0;
    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public final PID pidLinear = new PID(0.005,0.00000022,0.0000);
    public final PID pidAngular = new PID(0.08,0,0);


////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot(RobotMode robotMode, RobotAlliance robotAlliance, OpMode op) {
        super(robotMode, robotAlliance, op);

        odometry = new Odometry(op);
        drivetrain = new MecanumDrivetrain(op);
        teleSkope = new TeleSkope(op);

        messageTelemetry = new MessageTelemetry(op,this);

    }
    @Override
    // Метод инициализации того, чего надо
    public void init() {
        odometry.init();
        drivetrain.init();
        messageTelemetry.init();
    }


    public double returnDistance(double VelMax, double assel ){
        return Math.pow(VelMax, 2)/ (2* assel);
    }

    public double returnSpeed(Position position, double assel){
        return Math.sqrt(2 * position.toVector().length() * assel);
    }

    // Метод, обрабатывающий задачу перемещения робота в точку
    public TaskHandler driveToPosition = new TaskHandler() {
        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.driveStandartArgs args = (StandartArgs.driveStandartArgs) _args;

            int result;

            double assel;

            if(args.position.x > 0 || args.position.x < 0 && args.position.y == 0){
                assel = MAX_ACCEL_FRONT;
            } else if (args.position.x == 0 && args.position.y > 0 || args.position.y < 0) {
                assel = MAX_ACCEL_SIDE;
            }else {
                assel = Math.sqrt(Math.pow(MAX_ACCEL_FRONT, 2) + Math.pow(MAX_ACCEL_SIDE, 2));
            }

            double Vel;

            double VelMax = args.max_linear_speed;
            double Vel0 = returnSpeed(args.position, assel);

            if(args.position.toVector().length() > returnDistance(VelMax, assel)){
                Vel = VelMax;
            }else{
                Vel = Vel0;
            }

            double errorHeading = args.position.heading - odometry.getGlobalPosition().getHeading();
            Vector2 target = args.position.toVector();

            target.sub(odometry.getGlobalPosition().toVector());

            double speedPID = pidLinear.calculate(Vel, odometry.getSpeed());
            double angularPID = pidAngular.calculate(args.max_angular_speed, odometry.getAngularVelocity()); //Всегда положителен

            double headingVel =  errorHeading * angularPID;

            Vector2 targetVel = new Vector2(target);

            targetVel.multyplie(speedPID);

            targetVel.normalize();

//            if(Math.abs(targetVel.length()) < 0.10){
//                targetVel.setVectorLength((targetVel.length()/Math.abs(targetVel.length()) * 0.1));
//            } else if (Math.abs(headingVel) < 0.11) {
//                headingVel = (headingVel/Math.abs(headingVel)) * 0.11;
//            }

            drivetrain.setVelocity(targetVel, headingVel);

            messageTelemetry.telemetry.update();

            if(Math.abs(target.length()) < 3){
                drivetrain.offMotors();
                result = 0;
            }else {
                result = -1;
            }

            telemetry();

            return result;
        }
    };

    // Метод, обрабатывающий задачу подъема телескопа
    public TaskHandler setTeleskopePos = new TaskHandler() {
        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.teleskopeStandartArgs args = (StandartArgs.teleskopeStandartArgs) _args;
            int result = -1;

            // TODO: обработчик застреваний телескопа
            //  если робот вдруг поехал
            //  если телескоп не поднялся на нужный уровень и стоит на месте долго

            return result;
        }
    };


    public void telemetry(){
        if(robotMode == RobotMode.TELEOP){
            messageTelemetry.dataForTeleOp();
        }else if(robotMode == RobotMode.AUTO){
            messageTelemetry.dataForAuto();
        }
    }

    // Gamepad 1
    @Override
    public void teleopPl1() {
        time.milliseconds();

        double velocityAngle = (((odometry.encL.getVelocity() + odometry.encR.getVelocity())/2)/ TICK_PER_CM)/(DIST_BETWEEN_ENC_X/2);//рад/сек
        double velocityX = (((odometry.encL.getVelocity() - odometry.encR.getVelocity())/2)/TICK_PER_CM);// см/сек
        double velocityY = (odometry.encM.getVelocity()/TICK_PER_CM - (velocityAngle * OFFSET_ENC_M_FROM_CENTER));// см/сек

        double targetVelX = op.gamepad1.left_stick_y * MAX_CM_PER_SEC;
        double targetVelY = op.gamepad1.left_stick_x * MAX_CM_PER_SEC;
        double targetAngleVel = op.gamepad1.right_stick_x * MAX_RAD_PER_SEC;

        if(targetAngleVel != 0){
            oldTime = (time.milliseconds()/1000) - oldTime;
            if(maxTime < oldTime){
                maxTime = oldTime;
            }
            if(maxVelAAngle < odometry.getAngularVelocity()){
                maxVelAAngle = odometry.getAngularVelocity();
                timeVel = oldTime;
                headingWhenReachMaxVel = odometry.getGlobalPosition().heading;
            }
            if(maxAngleAccel < odometry.getAngularAcceleration()){
                maxAngleAccel = odometry.getAngularAcceleration();
                timeAngle = oldTime;
                headingWhenReachMaxAngle = odometry.getGlobalPosition().heading;
            }
        }else {
            oldTime = 0;
            time.reset();
        }

        double maxSpeed = 1;

        double kF = maxSpeed/MAX_CM_PER_SEC;//макс см/сек
        double kFR = maxSpeed/MAX_RAD_PER_SEC;//макс рад/сек
        double kP = -0.00;//коеф торможения робота

        double forwardVoltage = (targetVelX - velocityX) * kP + targetVelX * kF;
        double sideVoltage = (targetVelY - velocityY) * kP + targetVelY * kF;
        double angleVoltage = (targetAngleVel - velocityAngle) * kP + targetAngleVel * kFR;

        drivetrain.setVelocityTeleOp(forwardVoltage, sideVoltage, angleVoltage);

        messageTelemetry.addData("maxVelAAngle", maxVelAAngle);
        messageTelemetry.addData("timeVel", timeVel);
        messageTelemetry.addData("headingWhenReachMaxVel", headingWhenReachMaxVel);
        messageTelemetry.telemetry.addLine();
        messageTelemetry.addData("maxAngleAccel", maxAngleAccel);
        messageTelemetry.addData("timeAngle", timeAngle);
        messageTelemetry.addData("headingWhenReachMaxAngle", headingWhenReachMaxAngle);
        messageTelemetry.telemetry.addLine();
        messageTelemetry.addData("maxTime", maxTime);
        messageTelemetry.addData("AngleAccel", odometry.getAngularAcceleration());
        messageTelemetry.addData("Heading", odometry.getGlobalPosition().heading);
        messageTelemetry.addData("ticks encoder left", odometry.encL.getCurrentPosition());
        messageTelemetry.addData("ticks encoder right", odometry.encR.getCurrentPosition());
        messageTelemetry.addData("X", odometry.getGlobalPosition().x);
        messageTelemetry.addData("Y", odometry.getGlobalPosition().y);

        //ТЕЛЕМЕТРИЯ
//        messageTelemetry.setTargetVel(targetVelX, targetVelY, targetAngleVel, "см/сек", "рад/сек");
//        messageTelemetry.setKoefForDrives(kF, kFR);
//        messageTelemetry.setTargetVoltage(forwardVoltage, sideVoltage, angleVoltage);
    }

    // Gamepad 2
    @Override
    public void teleopPl2() {
        double upStandingVel = op.gamepad2.right_stick_x;
        double horizontalVel = op.gamepad2.left_stick_x;//Привести в правильное числовое значения для моторов

//        teleSkope.setVelHorizontalTeleOp(horizontalVel);//продумать логику о пропорциональном движении телескопов
//        teleSkope.setVelUpStandingTeleOp(upStandingVel);
    }
}
