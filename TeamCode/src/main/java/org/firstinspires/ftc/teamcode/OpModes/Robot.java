package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.TeleSkope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual.DataDisplayer;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual.Metry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataFilter;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataObject;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataTarget;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.JoystickStatement;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
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
    public final Metry metry;
    public final Joysticks joysticks;
    public final DataDisplayer dataDisplayer;

    boolean switchableH,isHeadless, isCruise, switchableC, switchableP, isProp = true;
    double releasedH, releasedC, releasedP;
    double horizontalPos = 0.43, forwardC, sideC;

    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public final PID pidLinearX = new PID(0.018,0.00000022,0.0000, -1,1);
    public final PID pidLinearY = new PID(0.018,0.00000022,0.0000, -1,1);
    public final PID pidAngular = new PID(0.93,0.000018,0, -1,1);

////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot(RobotMode robotMode, RobotAlliance robotAlliance, OpMode op) {
        super(robotMode, robotAlliance, op);

        metry = new Metry(op);
        joysticks = new Joysticks(op);
        odometry = new Odometry(op);
        drivetrain = new MecanumDrivetrain(op);
        teleSkope = new TeleSkope(op);

        dataDisplayer = new DataDisplayer(this);
    }
    @Override
    // Метод инициализации того, чего надо
    public void init() {
        odometry.init();
        drivetrain.init();
        teleSkope.init();
        joysticks.init();
        metry.init();
        dataDisplayer.init();
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
            int result;

                boolean errorPosDone = false;
                boolean errorHeadingDone = false;

                double linearVel; // Линейная скорость робота

                Vector2 errorPos = new Vector2();

                // Находим ошибку положения
                errorPos.x = args.position.getX() - odometry.getGlobalPosition().getX();
                errorPos.y = args.position.getY() - odometry.getGlobalPosition().getY();
                double errorHeading = args.position.getHeading() - odometry.getGlobalPosition().getHeading();

                // Направление движения
                Vector2 targetVel = new Vector2(errorPos);
                targetVel.normalize();
                targetVel.rotate(-odometry.getGlobalPosition().getHeading()); // Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!

                // Выбираем скорости в зависимости от величины ошибки
                if (errorPos.length() > returnDistance(args.max_linear_speed, MAX_LINEAR_ACCEL)) {
                    linearVel = args.max_linear_speed; //Максимально допустимая скорость с args
                } else {
                    linearVel = MIN_LINEAR_SPEED;
                }

                if(Math.abs(targetVel.y) > MAX_LINEAR_SIDE){
                    targetVel.multyplie(MAX_LINEAR_SIDE/Math.abs(targetVel.y));
                }

                if (linearVel < MIN_LINEAR_SPEED) linearVel = MIN_LINEAR_SPEED;// Ограничиваем скорость снизу

                if (errorPos.length() < 2){
                    errorPosDone = true;
                    linearVel = 0;
                }

                if(Math.abs(errorHeading) < Math.toRadians(1.5)){
                    errorHeadingDone = true;
                }

                targetVel.multyplie(linearVel);

                // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
                double speedPIDX = pidLinearX.calculate(targetVel.x, odometry.getVelocity().x);
                double speedPIDY = pidLinearY.calculate(targetVel.y, odometry.getVelocity().y);
                double angularPID = pidAngular.calculate(args.position.getHeading(), odometry.getGlobalPosition().getHeading());

                if(errorPosDone && errorHeadingDone){
                    result = 0;
                    drivetrain.offMotors();
                }else{
                    result = -1;
                    drivetrain.setXYHeadVel(speedPIDX, speedPIDY, angularPID);
                }

                dataDisplayer.addData("Оставшийся угол", errorHeading);
                dataDisplayer.addData("Оставшийся расстояние", errorPos.length());
                dataDisplayer.addData("Оставшийся X", errorPos.x);
                dataDisplayer.addData("Оставшийся Y", errorPos.y);

                dataDisplayer.update();

            return result;
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
            dataDisplayer.dataForTeleOp();
        }else if(robotMode == RobotMode.AUTO){
            dataDisplayer.dataForAuto();
        }
    }

    // Gamepad 1
    @Override
    public void teleopPl1() {
        Gamepad g1 = joysticks.getGamepad1();

        double velocityAngle = -(((odometry.getEncL().getVelocity() + odometry.getEncR().getVelocity()))/ TICK_PER_CM)/DIST_BETWEEN_ENC_X;//рад/сек
        double velocityX = (((odometry.getEncL().getVelocity() - odometry.getEncR().getVelocity())/2)/TICK_PER_CM);// см/сек
        double velocityY = (odometry.getEncM().getVelocity()/TICK_PER_CM - (velocityAngle * OFFSET_ENC_M_FROM_CENTER));// см/сек

        double targetVelX = -g1.left_stick_y * MAX_CM_PER_SEC;
        double targetVelY = g1.left_stick_x * MAX_CM_PER_SEC;
        double targetAngleVel = g1.right_stick_x * MAX_RAD_PER_SEC;

        double maxSpeed = 1;

        double kF = maxSpeed/MAX_CM_PER_SEC;//макс см/сек
        double kFR = maxSpeed/MAX_RAD_PER_SEC;//макс рад/сек
        double kP = -0.00;//коеф торможения робота

        if(g1.a && g1.y && releasedH == 0) {
            switchableH = !switchableH;
            releasedH = 1;}

        if(!g1.a && !g1.y && releasedH != 0){
            releasedH = 0;
        }

        if(g1.x && releasedC == 0) {
            switchableC = !switchableC;
            releasedC = 1;}

        if(!g1.x && releasedC != 0){
            releasedC = 0;
        }

        isHeadless = switchableH;

        isCruise = switchableC;

        double forwardVoltage = (targetVelX - velocityX) * kP + targetVelX * kF;
        double sideVoltage = (targetVelY - velocityY) * kP + targetVelY * kF;
        double angleVoltage = (targetAngleVel - velocityAngle) * kP + targetAngleVel * kFR;

        if(isHeadless){
            double k = odometry.getGlobalPosition().getHeading()/Math.toRadians(90);

            boolean ifForward = Math.abs(forwardVoltage) > Math.abs(sideVoltage);
            boolean ifSide = Math.abs(sideVoltage) > Math.abs(forwardVoltage);

            dataDisplayer.addData("k", k);

            if(k > 0){
                if(ifForward) {
                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1/k);
                } else if (ifSide) {
                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1/k, 1);
                }else {
                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
                }
            }else if (k < 0){
                if(ifForward) {
                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1/k, 1);
                } else if (ifSide) {
                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1/k);
                }else {
                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
                }
            }else{
                drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
            }
        }

        if(isCruise){
            forwardC = Range.clip(forwardC + (-g1.left_stick_y/18), -0.4, 0.4);
            sideC = Range.clip(sideC + (g1.left_stick_x/18), -0.4, 0.4);

            drivetrain.setPowerTeleOp(forwardC, sideC, angleVoltage);
        }else {
            drivetrain.setPowerTeleOp(forwardVoltage, sideVoltage, angleVoltage);
        }

    }

    // Gamepad 2
    @Override
    public void teleopPl2() {
        Gamepad g2 = joysticks.getGamepad2();

        double upStandingVel = -g2.right_stick_y;

        horizontalPos = Range.clip(horizontalPos + (-g2.left_stick_y/18),0.05,0.43);

        if(g2.x && releasedP == 0) {
            switchableP = !switchableP;
            releasedP = 1;}

        if(!g2.x && releasedP != 0){
            releasedP = 0;
        }

        isProp = switchableP;

        teleSkope.setTeleskope(upStandingVel, horizontalPos, isProp);

        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.ENCL, DataFilter.CM);
        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.UPSTANDINGLEFT, DataFilter.CM);
        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.UPSTANDINGRIGHT, DataFilter.CM);
        dataDisplayer.showValue(DataTarget.displayOtherPosition, DataObject.HORIZONTAL, DataFilter.POSITION);

        dataDisplayer.addData("horizontalPos", horizontalPos);
        dataDisplayer.showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD2, JoystickStatement.LEFT_STICK);
    }
}
