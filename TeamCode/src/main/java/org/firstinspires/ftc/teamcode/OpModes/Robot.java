package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

public class Robot extends RobotCore implements CONSTS{
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Системы робота.
    // Железо хранится уже в самой системе.
    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота
    public final TeleSkope teleSkope;
    public final MessageTelemetry messageTelemetry;

    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public final PID pidLinear = new PID(0.03,0.001,0.05);
    public final PID pidAngular = new PID(0.1,0,0);


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
            double Vel0 = odometry.returnSpeed(args.position, assel);

            if(args.position.toVector().length() > odometry.returnDistance(VelMax, assel)){
                Vel = VelMax;
            }else{
                Vel = Vel0;
            }

            double errorHeading = args.position.heading - odometry.getGlobalPosition().getHeading();
            Vector2 target = args.position.toVector();

            target.sub(odometry.getGlobalPosition().toVector());

            Vector2 targetVel = new Vector2(target);
            targetVel.normalize();

            double speedPID = pidLinear.calculate(Vel, odometry.getSpeed());
            double angularPID = pidAngular.calculate(args.max_angular_speed, odometry.getAngularVelocity()); //Всегда положителен

            double headingVel =  errorHeading * angularPID;

            targetVel.multyplie(speedPID);

            drivetrain.setVelocity(targetVel, headingVel);

            messageTelemetry.telemetry.update();

            if(odometry.getAcceleration().length() == 0 && targetVel.length() == 0){
                drivetrain.offMotors();
                result = 0;
            }else{
                result = -1;
            }

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

    @Override
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
        double velocityAngle = (((odometry.encL.getVelocity() + odometry.encR.getVelocity())/2)/ TICK_PER_CM);//см/сек
        double velocityX = (((odometry.encL.getVelocity() - odometry.encR.getVelocity())/2)/TICK_PER_CM);// см/сек
        double velocityY = (odometry.encM.getVelocity()/TICK_PER_CM - (velocityAngle * OFFSET_ENC_M_FROM_CENTER))/(DIST_BETWEEN_ENC_X/2);// рад/сек

        double targetVelX = op.gamepad1.left_stick_y * MAX_TPS_ENCODER/TICK_PER_CM;
        double targetVelY = op.gamepad1.left_stick_x * MAX_TPS_ENCODER/TICK_PER_CM;
        double targetAngleVel = op.gamepad1.right_stick_x * MAX_CM_PER_SEC/(DIST_BETWEEN_ENC_X/2);

        double maxSpeed = 1;

        double kF = maxSpeed/MAX_CM_PER_SEC;//макс см/сек
        double kFR = maxSpeed/MAX_RAD_PER_SEC;//макс рад/сек
        double kP = -0.0001;//коеф торможения робота

        double forwardVoltage = (targetVelX - velocityX) * kP + targetVelX* kF;
        double sideVoltage = (targetVelY - velocityY) * kP +  targetVelY* kF;
        double angleVoltage = (targetAngleVel - velocityAngle) * kP +  targetAngleVel* kFR;

        drivetrain.setVelocityTeleOp(forwardVoltage, sideVoltage, angleVoltage);

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
