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
    boolean switchable,isHeadless;
    double released;
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

    // Метод, обрабатывающий задачу перемещения робота в точку
    public TaskHandler driveToPosition = new TaskHandler() {
        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.driveStandartArgs args = (StandartArgs.driveStandartArgs) _args;



            telemetry();

            return 0;
        }

        private double returnDistance(double VelMax, double assel ){
            return Math.pow(VelMax, 2) / (2 * assel);
        }

        private double returnSpeed(Position position, double assel){
            return Math.sqrt(2 * position.toVector().length() * assel);
        }
    };

    // Метод, обрабатывающий задачу подъема телескопа
    public TaskHandler setTeleskopePos = new TaskHandler() {
        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }
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

        double velocityAngle = -(((odometry.encL.getVelocity() + odometry.encR.getVelocity()))/ TICK_PER_CM)/DIST_BETWEEN_ENC_X;//рад/сек
        double velocityX = (((odometry.encL.getVelocity() - odometry.encR.getVelocity())/2)/TICK_PER_CM);// см/сек
        double velocityY = (odometry.encM.getVelocity()/TICK_PER_CM - (velocityAngle * OFFSET_ENC_M_FROM_CENTER));// см/сек

        double targetVelX = op.gamepad1.left_stick_y * MAX_CM_PER_SEC;
        double targetVelY = op.gamepad1.left_stick_x * MAX_CM_PER_SEC;
        double targetAngleVel = op.gamepad1.right_stick_x * MAX_RAD_PER_SEC;

        double maxSpeed = 1;

        double kF = maxSpeed/MAX_CM_PER_SEC;//макс см/сек
        double kFR = maxSpeed/MAX_RAD_PER_SEC;//макс рад/сек
        double kP = -0.00;//коеф торможения робота

        if(op.gamepad1.a && op.gamepad1.y && released == 0) {
            switchable = !switchable;
            released = 1;}

        if(!op.gamepad1.a && !op.gamepad1.y && released != 0){
            released = 0;
        }

        isHeadless = switchable;

        double forwardVoltage = (targetVelX - velocityX) * kP + targetVelX * kF;
        double sideVoltage = (targetVelY - velocityY) * kP + targetVelY * kF;
        double angleVoltage = (targetAngleVel - velocityAngle) * kP + targetAngleVel * kFR;

        if(isHeadless){
            double k = odometry.getGlobalPosition().heading/Math.toRadians(90);

            boolean ifForward = Math.abs(forwardVoltage) > Math.abs(sideVoltage);
            boolean ifSide = Math.abs(sideVoltage) > Math.abs(forwardVoltage);
            messageTelemetry.addData("k", k);
            if(k > 0){
                if(ifForward) {
                    drivetrain.setVelocityTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, k, 1/k);
                } else if (ifSide) {
                    drivetrain.setVelocityTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1/k, k);
                }else {
                    drivetrain.setVelocityTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
                }
            }else if (k < 0){
                if(ifForward) {
                    drivetrain.setVelocityTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1/k, k);
                } else if (ifSide) {
                    drivetrain.setVelocityTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, k, 1/k);
                }else {
                    drivetrain.setVelocityTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
                }
            }else{
                drivetrain.setVelocityTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
            }
        }else{
            drivetrain.setVelocityTeleOp(forwardVoltage, sideVoltage, angleVoltage);
        }
        messageTelemetry.addData("gamepadY", op.gamepad1.left_stick_y);
        messageTelemetry.addData("X", odometry.getGlobalPosition().x);
        messageTelemetry.addData("Y", odometry.getGlobalPosition().y);
        messageTelemetry.addData("heading", odometry.getGlobalPosition().heading);
        messageTelemetry.addData("encM", odometry.encM.getCurrentPosition());
        messageTelemetry.addData("encL", odometry.encL.getCurrentPosition());
        messageTelemetry.addData("encR", odometry.encR.getCurrentPosition());

        messageTelemetry.addData("deltaRad", odometry.deltaPosition.heading);
        messageTelemetry.addData("deltaY", odometry.deltaPosition.y);
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
