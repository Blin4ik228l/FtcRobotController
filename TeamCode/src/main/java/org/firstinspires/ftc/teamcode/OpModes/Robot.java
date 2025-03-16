package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.ServosService;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups.TeleSkope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Singles.Button;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual.Metry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.RobotModuleStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.PID;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

public class Robot extends RobotCore implements Consts, ConstsTeleskope {
////////////////////////////////////////////////////////////////////////////////////////////////////

    double horizontalPos = CLOSE_POS_HORIZONTAL2;

    // Системы робота.
    // Железо хранится уже в самой системе.
    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота
    public final TeleSkope teleSkope;
    public final Metry metry;
    public final Joysticks joysticks;
    public final ServosService servosService;
    public final Button button;

    public RobotModuleStatus robotStatus;
    public RobotModuleStatus teleskopeStatus;
//    public final RGBColorSensor colorSensor;
//    public final Distance distanceSensor;
//    public final TaskManager taskManager;

    Gamepad g1;

    Gamepad g2;
    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    double I = 0.75;
    double kPang = 1;
    double kPlin = 0.024;
    public final PID pidLinearX = new PID(kPlin,0.000000080,0.000, -I,I);
    public final PID pidLinearY = new PID(kPlin,0.000000080,0.000, -I,I);
    public final PID pidAngular = new PID(kPang,0.0000000,0.00, -I,I);

////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot(RobotMode robotMode, RobotAlliance robotAlliance, OpMode op, Position startPos) {
        super(robotMode, robotAlliance, op);

        metry = new Metry(op);
        joysticks = new Joysticks(op);
        odometry = new Odometry(op, startPos);
        drivetrain = new MecanumDrivetrain(op);
        servosService = new ServosService(op);
        teleSkope = new TeleSkope(op, servosService);
        button = new Button(op);

//        this.robotStatusHandler.robot = this;

//        colorSensor = new RGBColorSensor(op);
//        distanceSensor = new Distance(op);

    }
    @Override
    // Метод инициализации того, чего надо
    public void init() {
        op.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        op.telemetry.setItemSeparator("");

        odometry.init();
        drivetrain.init();
        servosService.init();
        teleSkope.init();
        joysticks.init();
        button.init();

        g1 = joysticks.getGamepad1();
        g2 = joysticks.getGamepad2();

        robotStatus = RobotModuleStatus.Normal;
        teleskopeStatus = RobotModuleStatus.Normal;
//        colorSensor.init();
//        distanceSensor.init();
    }

    public void autoInterrupt(){
        odometry.interrupt();
        robotStatusHandler.interrupt();
    }

    public void teleInterrupt(){
        odometry.interrupt();
//        taskManager.tele1.interrupt();
//        taskManager.tele2.interrupt();
        taskManager.interrupt();
    }


    // Метод, обрабатывающий задачу перемещения робота в точку
    public TaskHandlerOrdinal driveToPosition = new TaskHandlerOrdinal() {
        public ElapsedTime delayTime = new ElapsedTime();

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.driveArgs args = (StandartArgs.driveArgs) _args;

            if(delayTime.seconds() <= args.delayTime){
                return -1;
            }

            double progressBar;

            double progressToPos;
            double progressHeading;

            int result;

            boolean errorPosDone = false;
            boolean errorHeadingDone = false;

            double linearVel; // Линейная скорость робота

            Vector2 errorPos = new Vector2();

            // Находим ошибку положения
            errorPos.x = args.position.getX() - odometry.getGlobalPosition().getX();
            errorPos.y = args.position.getY() - odometry.getGlobalPosition().getY();
            double errorHeading = args.position.getHeading() - odometry.getGlobalPosition().getHeading();

            progressHeading = (args.position.getHeading() - errorHeading)/args.position.getHeading();
            progressToPos = ((args.position.getX() - errorPos.x)/args.position.getX() + (args.position.getY() - errorPos.y)/args.position.getY()) / 2.0;
            progressBar = (progressHeading + progressToPos) / 2;

            // Направление движения
            Vector2 targetVel = new Vector2(errorPos);
            targetVel.normalize();
            targetVel.rotate(-odometry.getGlobalPosition().getHeading()); // Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!

            // Выбираем скорости в зависимости от величины ошибки
            linearVel = errorPos.length() > returnDistance(args.max_linear_speed, MAX_LINEAR_ACCEL) ? args.max_linear_speed : MIN_LINEAR_SPEED;

//            if(linearVel <= MID_LINEAR_SPEED){
//                midSpeed -= 1;
//            }


            if(linearVel < MIN_LINEAR_SPEED) linearVel = MIN_LINEAR_SPEED;// Ограничиваем скорость снизу

            if(Math.abs(errorHeading) < Math.toRadians(8)){
                kPang += 0.45;
            } else if (Math.abs(errorHeading) >= Math.toRadians(8) && Math.abs(errorHeading) < Math.toRadians(15)) {
                kPang = 0.50;
            } else{
                kPang = 1;
            }


            if (errorPos.length() < 0.5){
                errorPosDone = true;
                linearVel = 0;
            }

            if(Math.abs(errorHeading) < Math.toRadians(0.5) ){
                errorHeadingDone = true;
                kPang = 0.0;
            }

            pidAngular.setPID(kPang, 0,0);

            targetVel.multyplie(linearVel);

            // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
            double speedPIDX = pidLinearX.calculate(targetVel.x, odometry.getVelocity().x);
            double speedPIDY = pidLinearY.calculate(targetVel.y, odometry.getVelocity().y);
            double angularPID = pidAngular.calculate(args.position.getHeading(), odometry.getGlobalPosition().getHeading());

            if(errorPosDone && errorHeadingDone){
                result = 0;
                delayTime.reset();
                drivetrain.offMotors();
            }else{
                result = -1;

                drivetrain.setXYHeadVel(speedPIDX, speedPIDY, angularPID);
            }

            op.telemetry.addLine("driveToPosition")
                    .addData("\nkPang", kPang)
                    .addData("\nВектор", targetVel.length())
                    .addData("\nВектор X", targetVel.x)
                    .addData("\nВектор Y", targetVel.y)
                    .addData("\nОставшийся угол", errorHeading * 57.29)
                    .addData("\nangularPID", angularPID)
                    .addData("\nspeedPIDX", speedPIDX)
                    .addData("\nspeedPIDY", speedPIDY)
                    .addData("\nОставшийся расстояние", errorPos.length())
                    .addData("\nОставшийся X", errorPos.x)
                    .addData("\nОставшийся Y", errorPos.y)
                    .addData("\nresult", result);
            op.telemetry.update();

            drivetrain.getMotorsPower();

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
    public TaskHandlerOrdinal setVerticalTeleskopePos = new TaskHandlerOrdinal() {

        public ElapsedTime delayTime = new ElapsedTime();

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {

            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.verticalArgs args = (StandartArgs.verticalArgs) _args;

            if(delayTime.seconds() <= args.delayTime){
                return -1;
            }

            int result;

            double target = args.teleskope_height - teleSkope.getHeight();

            if(Math.abs(target) < 1.8){
                result = 0;
                delayTime.reset();
                if(args.teleskope_height < 10){
                    teleSkope.offMotors();
                }else {
                    teleSkope.keepInPower();
                }
            }else {
                result = -1;

                teleSkope.setTeleskopeAuto(args.max_speed, args.teleskope_height);
            }

            op.telemetry.addLine("setVerticalTeleskopePos")
                    .addData("\nheight",teleSkope.getHeight())
                    .addData("\nresult", result);
            op.telemetry.update();

            return result;
        }
    };

    public TaskHandlerOrdinal setHorizontalTeleskopePos = new TaskHandlerOrdinal() {
        public ElapsedTime delayTime = new ElapsedTime();

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }
        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.horizontalArgs args = (StandartArgs.horizontalArgs) _args;

            if(delayTime.seconds() <= args.delayTime){
                return -1;
            }

            int result;

            boolean horizontalPosDone = false;

            teleSkope.setLeftRightHorizont(args.servo_pos);

            if(servosService.getLeft().getPosition() == Range.clip(args.servo_pos + CLOSE_POS_HORIZ_LEFT, CLOSE_POS_HORIZ_LEFT, OPEN_POS_HORIZ_LEFT)) {
                horizontalPosDone = true;
            }

            if(horizontalPosDone){
                result = 0;
                delayTime.reset();
            }else{
                result = -1;
            }

            op.telemetry.addLine("setHorizontalTeleskopePos")
                    .addData("\nservoPos", servosService.getLeft())
                    .addData("\nresult", result);
            op.telemetry.update();

            return result;
        }
    };

    public TaskHandlerOrdinal setZahvat = new TaskHandlerOrdinal(){
        public ElapsedTime delayTime = new ElapsedTime();

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.captureArgs args = (StandartArgs.captureArgs) _args;

            if (delayTime.seconds() < args.delayTime){
                return -1;
            }

            int result;

            boolean flipDone = false;
            boolean hookDone = false;

            teleSkope.setFlip(args.flipPos);
            teleSkope.setHook(args.hookPos);

            if(servosService.getFlip().getPosition() == args.flipPos){
                flipDone = true;
            }

            if(servosService.getHook().getPosition() == args.hookPos){
                hookDone = true;
            }

            if(flipDone && hookDone ){
                result = 0;
                delayTime.reset();
            }else{
                result = -1;
            }

            op.telemetry.addLine("SetZahvat")
                    .addData("\nhookPose", servosService.getHook().getPosition())
                    .addData("\nflipPose", servosService.getFlip().getPosition())
                    .addData("\nresult", result);
            op.telemetry.update();

            return result;
        }
    };

    // Gamepad 1
    @Override
    public synchronized void teleopPl1() {
        double max_speed = 0.8;
        double accelLinear = 1.3, accelAngle = 1.3;

        if(g1.left_trigger > 0.05 && g1.right_trigger < 0.05){//Ускорение робота
            accelLinear = 1.8;
            accelAngle = 1.8;
        }

        if(g1.right_trigger > 0.05 && g1.left_trigger < 0.05){//Замедление робота
            accelLinear = 0.25;
            accelAngle = 0.25;
        }

        double forward = -1*g1.left_stick_y;
        double side = g1.left_stick_x;
        double turn = g1.right_stick_x;

        if (Math.abs(forward) < 0.1 && forward != 0){
            forward += 0.1 * Math.signum(forward);
        }
        if (Math.abs(side) < 0.1 && side != 0){
            side += 0.15 * Math.signum(side);
        }

        double forwardVoltage = Range.clip(forward * accelLinear , -max_speed, max_speed);
        double sideVoltage    = Range.clip(side * accelLinear ,  -max_speed, max_speed);
        double angleVoltage   = Range.clip(turn * accelAngle, -max_speed, max_speed);

        drivetrain.setPowerTeleOp(forwardVoltage, sideVoltage, angleVoltage);
    }

    // Gamepad 2
    @Override
    public synchronized void teleopPl2() {
        double leftStickY = -g2.left_stick_y;

        double upStandingVel = -g2.right_stick_y;

        teleSkope.setTeleskopeTele(upStandingVel, horizontalPos, joysticks);

        switch (joysticks.getDpadUp(g2.dpad_down)){
            case 0:
                horizontalPos = CLOSE_POS_HORIZONTAL2;
                break;

            case 1:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.05;
                break;

            case 2:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.1;
                break;

            case 3:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.15;
                break;

            case 4:
                horizontalPos = CLOSE_POS_HORIZONTAL2 + 0.2;
                break;

            case 5:
                horizontalPos = OPEN_POS_HORIZONTAL2;
                break;
        }

        switch (joysticks.getGearTele()) {
            case 0:
                teleSkope.setTeleskopeHeight(0, joysticks);
                break;

            case 1:
                teleSkope.setTeleskopeHeight(17, joysticks);
                break;

            case 2:
                teleSkope.setTeleskopeHeight(55, joysticks);
                break;

//            case 3:
//                teleSkope.setTeleskopeHeight(70, joysticks);
//                break;
        }

        if(joysticks.isY_G2())
        {
            teleSkope.setFlip(ConstsTeleskope.HANG_POS_FLIP);
            joysticks.isB_g2 = false;
        }
        if (joysticks.isB_G2() )
        {
            teleSkope.setFlip(ConstsTeleskope.TAKE_POS_FLIP);
            joysticks.isY_g2 = false;
        }
        if (!joysticks.isB_G2() && !joysticks.isY_G2())
        {
            teleSkope.setFlip(ConstsTeleskope.MIDLE_POS_FLIP);
        }


        if (joysticks.isA_G2()){
            if(servosService.getHook().getPosition() == OPEN_POS_HOOK)
            {
                servosService.getHook().setPosition(CLOSE_POS_HOOK);
            }else
            {
                teleSkope.setHook(ConstsTeleskope.OPEN_POS_HOOK);
            }
            joysticks.isA_g2 = false;
        }

        if(joysticks.getGamepad2().x) {
            while (joysticks.getGamepad2().x) {
                teleSkope.setVelUpStandingTeleOp(-0.7);
            }
            servosService.getHook().setPosition(OPEN_POS_HOOK);
            teleSkope.setTeleskopeHeight(17, joysticks);
            joysticks.gearTele = 1;
        }

    }

}
