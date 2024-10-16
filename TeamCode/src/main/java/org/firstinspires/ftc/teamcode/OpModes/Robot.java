package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.Utils.PID;
import org.firstinspires.ftc.teamcode.Utils.Vector2;

public class Robot extends RobotCore {
////////////////////////////////////////////////////////////////////////////////////////////////////

    // Системы робота.
    // Железо хранится уже в самой системе.
    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота

    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public final PID pidDriveTrainLinear = new PID(0,0,0);
    public final PID pidDriveTrainAngular = new PID(0,0,0);

////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot(RobotMode robotMode, RobotAlliance robotAlliance, HardwareMap hardwareMap) {
        super(robotMode, robotAlliance, hardwareMap);

        odometry = new Odometry(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap);
    }

    @Override
    // Метод инициализации того, чего надо
    public void init() {
        odometry.init();
        drivetrain.init();
    }

    // Метод, обрабатывающий задачу перемещения робота в точку
    public TaskHandler driveToPosition = new TaskHandler() {
        @Override
        public int execute(TaskManager thisTaskManager, StdArgs _args) {
            StdArgs.driveStdArgs args = (StdArgs.driveStdArgs) _args;
            int result;

            double errorHeading = args.position.heading - odometry.getGlobalPosition().getHeading();
            Vector2 errorPos = args.position.toVector();
            errorPos.sub(odometry.getGlobalPosition().toVector());

            Vector2 velocity = new Vector2(errorPos);
            velocity.normalize();

            double speedPID = pidDriveTrainLinear.calculate(args.max_linear_speed, odometry.getSpeed());
            double angularPID = pidDriveTrainAngular.calculate(args.max_angular_speed, odometry.getAngularVelocity()); //Всегда положителен

            velocity.multyplie(speedPID);

            drivetrain.setVelocity(velocity, angularPID);

            if(odometry.getAcceleration().mag() == 0){
                result = -1;
            }else{
                drivetrain.brakeMotors();
                result = 0;
            }

            return result;
        }
    };

    // Метод, обрабатывающий задачу подъема телескопа
    public TaskHandler setTeleskopePos = new TaskHandler() {
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

    // Gamepad 1
    @Override
    public void teleopPl1() {
        double velocityAngle = (((odometry.encL.getVelocity() + odometry.encR.getVelocity())/2)/ CONSTS.TICK_PER_CM);// см/сек
        double velocityX = (((odometry.encL.getVelocity() - odometry.encR.getVelocity())/2)/CONSTS.TICK_PER_CM);// см/сек
        double velocityY = (odometry.encM.getVelocity()/CONSTS.TICK_PER_CM/(CONSTS.DIST_BETWEEN_ENC_X/2)) - ((velocityAngle * CONSTS.OFFSET_ENC_M_FROM_CENTER)/(CONSTS.DIST_BETWEEN_ENC_X/2));// рад/сек

        double xVel = op.gamepad1.left_stick_x * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM;
        double yVel = op.gamepad1.left_stick_y * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM;
        double headingVel = op.gamepad1.right_stick_x * CONSTS.MAX_TPS_ENCODER/CONSTS.TICK_PER_CM/(CONSTS.DIST_BETWEEN_ENC_X/2);

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

    // Gamepad 2
    @Override
    public void teleopPl2() {
    }
}
