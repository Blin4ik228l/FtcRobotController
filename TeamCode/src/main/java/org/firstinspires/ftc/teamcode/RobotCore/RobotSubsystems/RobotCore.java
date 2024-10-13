package org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskCallback;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskExecMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.Utils.Position;
import org.firstinspires.ftc.teamcode.Utils.Vector2;

public class RobotCore implements Subsystem{

//  ПОЛЯ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Менеджер задач робота
    public final TaskManager taskManager;

    // Системы робота.
    // Железо хранится уже в самой системе.
    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота

    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public final PID pidDriveTrainLinear = new PID(0,0,0);
    public final PID pidDriveTrainAngular = new PID(0,0,0);

//  МЕТОДЫ КЛАССА
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Конструктор класса
    public RobotCore(TaskExecMode taskExecMode) {
        odometry = new Odometry();
        drivetrain = new MecanumDrivetrain();
        taskManager = new TaskManager(taskExecMode, this);
    }

    @Override
    // Метод инициализации того, чего надо
    public void init() {
        odometry.init();
        drivetrain.init();
    }

//  МЕТОДЫ, ОБРАБАТЫВАЮЩИЕ ЗАДАЧИ
////////////////////////////////////////////////////////////////////////////////////////////////////

    /** Как написать свой метод-обработчик задачи?
     *
     */
    public TaskCallback example = new TaskCallback() {
        @Override
        public int execute(TaskManager thisTaskManager, StdArgs _args) {
            // Какое-то действие робота
            return -1;
        }
    };

    // Метод, обрабатывающий задачу перемещения робота в точку
    public TaskCallback driveToPosition = new TaskCallback() {
        @Override
        public int execute(TaskManager thisTaskManager, StdArgs _args) {
            StdArgs.driveStdArgs args = (StdArgs.driveStdArgs) _args;
            int result;

            Vector2 errorPos = args.position.toVector();
            errorPos.sub(odometry.getGlobalPosition().toVector());

            Vector2 direction = new Vector2(errorPos);
            direction.normalize();

            double errorHeading = args.position.heading - odometry.getGlobalPosition().getHeading();
            double speedPID = pidDriveTrainLinear.calculate(args.max_linear_speed, odometry.getSpeed());

            double angularPID = pidDriveTrainAngular.calculate(args.max_angular_speed, odometry.getAngularVelocity());//Всегда положителен

            direction.multyplie(speedPID);
            errorHeading *= angularPID;

            drivetrain.setVelocity(direction, errorHeading);

            if(direction.x != odometry.getGlobalPosition().getX() && direction.y != odometry.getGlobalPosition().getY()
                    && errorHeading != odometry.getGlobalPosition().getHeading()){
                result = -1;
            }else{
                drivetrain.brakeMotors();
                result = 0;
            }

            return result;
        }
    };

    // Метод, обрабатывающий задачу подъема телескопа
    public TaskCallback setTeleskopePos = new TaskCallback() {
        @Override
        public int execute(TaskManager thisTaskManager, StdArgs _args) {
            StdArgs.teleskopeStdArgs args = (StdArgs.teleskopeStdArgs) _args;
            int result = -1;

            // TODO: обработчик застреваний телескопа
            //  если робот вдруг поехал
            //  если телескоп не поднялся на нужный уровень и стоит на месте долго

            return result;
        }
    };

    // Метод, обрабатывающий задачу телеопа
    public void teleop() {
        teleopPl1();
        teleopPl2();
    }

    public void teleopPl1() {
        double velocityAngle = (((odometry.encL.getVelocity() + odometry.encR.getVelocity())/2)/CONSTS.TICK_PER_CM);// см/сек
        double velocityX = (((odometry.encL.getVelocity() - odometry.encR.getVelocity())/2)/CONSTS.TICK_PER_CM);// см/сек
        double velocityY = (odometry.encM.getVelocity()/CONSTS.TICK_PER_CM/(CONSTS.DIST_BETWEEN_ENC_X/2)) - ((velocityAngle * CONSTS.OFFSET_ENC_M_FROM_CENTER)/(CONSTS.DIST_BETWEEN_ENC_X/2));// рад/сек

        double xVel = gamepad1.left_stick_x * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM;
        double yVel = gamepad1.left_stick_y * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM;
        double headingVel = gamepad1.right_stick_x * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM/(CONSTS.DIST_BETWEEN_ENC_X/2);

        double kF = 110/CONSTS.MAX_TPS_ENCODER;//макс см/сек
        double kP = -0.0001;
        double speed = 0.8;
        double kFR = speed/CONSTS.MAX_RAD_PER_SEC;//макс рад/сек

        double forward = (xVel - velocityX) * kP + xVel* kFR;
        double side = (yVel - velocityY) * kP +  yVel* kFR;
        double angle = (headingVel - velocityAngle) * kP +  headingVel* kFR;

        double rightFP = Range.clip((-forward - side - angle), -1.0, 1.0);
        double leftBP = Range.clip((forward + side - angle), -1.0, 1.0);
        double leftFP = Range.clip((forward - side - angle), -1.0, 1.0);
        double rightBP = Range.clip((-forward + side - angle), -1.0, 1.0);
    }

    public void teleopPl2() {

    }

}

