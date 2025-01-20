package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.ServosService;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.TeleSkope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual.Metry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.PID;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

public class Robot extends RobotCore implements CONSTS, CONSTSTELESKOPE {
////////////////////////////////////////////////////////////////////////////////////////////////////

    double horizontalPos = CLOSE_POS_HORIZONTAL;

    // Системы робота.
    // Железо хранится уже в самой системе.
    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота
    public final TeleSkope teleSkope;
    public final Metry metry;
    public final Joysticks joysticks;
    public final ServosService servosService;
//    public final RGBColorSensor colorSensor;
//    public final Distance distanceSensor;
//    public final TaskManager taskManager;

    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public final PID pidLinearX = new PID(0.018,0.0000001,0.000, -1,1);
    public final PID pidLinearY = new PID(0.018,0.0000001,0.000, -1,1);
    public final PID pidAngular = new PID(1.2,0.0,0.000, -1,1);
////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot(RobotMode robotMode, RobotAlliance robotAlliance, OpMode op) {
        super(robotMode, robotAlliance, op);

        metry = new Metry(op);
        joysticks = new Joysticks(op);
        odometry = new Odometry(op);
        drivetrain = new MecanumDrivetrain(op);
        servosService = new ServosService(op);
        teleSkope = new TeleSkope(op, servosService);
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


//        colorSensor.init();
//        distanceSensor.init();
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

            if(Math.abs(errorHeading) < Math.toRadians(2) && odometry.getAngularAcceleration() < 5 && odometry.getAngularVelocity() < 5){
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

            op.telemetry.addData("Вектор", targetVel.length());
            op.telemetry.addData("Вектор X", targetVel.x);
            op.telemetry.addData("Вектор Y", targetVel.y);
            op.telemetry.addData("Оставшийся угол", errorHeading * 57.29);
            op.telemetry.addData("angularPID", angularPID);
            op.telemetry.addData("speedPIDX", speedPIDX);
            op.telemetry.addData("speedPIDY", speedPIDY);
            drivetrain.getMotorsPower();
                op.telemetry.addData("Оставшийся расстояние", errorPos.length());
            op.telemetry.addData("Оставшийся X", errorPos.x);
            op.telemetry.addData("Оставшийся Y", errorPos.y);
//
            op.telemetry.addData("result", result);
//
            op.telemetry.update();

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
            StandartArgs.teleskopeStandartArgs args = (StandartArgs.teleskopeStandartArgs) _args;
            if(args.teleskope_height == 0){
                args.teleskope_height = 1;
            }
            return 0;
        }
        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            // TODO: обработчик застреваний телескопа
            //  если робот вдруг поехал
            //  если телескоп не поднялся на нужный уровень и стоит на месте долго
            StandartArgs.teleskopeStandartArgs args = (StandartArgs.teleskopeStandartArgs) _args;
            int result;

            double target = args.teleskope_height - teleSkope.getHeight();

            if(Math.abs(target) < 1){
                teleSkope.offMotors();
                result = 0;
            }else {
                teleSkope.setTeleskopePropAuto(args.max_speed, args.servo_pos, args.teleskope_height);
                result = -1;
            }

            return result;
        }
    };

    public TaskHandler setZahvat = new TaskHandler(){

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.zahvatStandartArgs args = (StandartArgs.zahvatStandartArgs) _args;
            int result;

            boolean flipDone = false;
            boolean hookDone = false;

            if(servosService.getFlip().getPosition() == args.flipPos){
                flipDone = true;
            }else{
                teleSkope.setFlip(args.flipPos);
            }

            if(servosService.getHook().getPosition() == args.hookPos){
                hookDone = true;
            }else{
                teleSkope.setHook(args.hookPos);
            }

            if(flipDone && hookDone){
                result = 0;
            }else{
                result = -1;
            }
            return result;
        }
    };

    // Gamepad 1
    @Override
    public synchronized void teleopPl1() {
        Gamepad g1 = joysticks.getGamepad1();

        double max_speed = 0.6;
        double accelLinear = 1.3, accelAngle = 1.3;

        if(g1.left_trigger > 0.05 && g1.right_trigger < 0.05){//Ускорение робота
            accelLinear = 1.8;
            accelAngle = 1.8;
        }

        if(g1.right_trigger > 0.05 && g1.left_trigger < 0.05){//Замедление робота
            accelLinear = 0.3;
            accelAngle = 0.3;
        }

        if(!joysticks.isUpGear()){
            if(joysticks.getGear() == 5){
                max_speed /= 1.2;
            } else if (joysticks.getGear() == 4) {
                max_speed /= 1.3;
            } else if (joysticks.getGear() == 3) {
                max_speed /= 1.4;
            }else{
                max_speed /= 1;
            }
        }

        if (joysticks.isUpGear()){
            if(joysticks.getGear() == 2){
                max_speed *= 1.2;
            } else if (joysticks.getGear() == 3) {
                max_speed *= 1.3;
            } else if (joysticks.getGear() == 4) {
                max_speed *= 1.4;
            }else {
                max_speed = 1;
            }
        }

        double forward = -1*g1.left_stick_y;
        double side = g1.left_stick_x;
        double turn = g1.right_stick_x;

        if (Math.abs(forward) < 0.12 && forward != 0){
            forward += 0.12 * Math.signum(forward);
        }
        if (Math.abs(side) < 0.12 && side != 0){
            side += 0.12 * Math.signum(side);
        }

        double forwardVoltage = Range.clip(forward * accelLinear , -max_speed, max_speed);
        double sideVoltage    = Range.clip(side * accelLinear ,  -max_speed, max_speed);
        double angleVoltage   = Range.clip(turn * accelAngle, -max_speed, max_speed);

        drivetrain.setPowerTeleOp(forwardVoltage, sideVoltage, angleVoltage);

        //        if(joysticks.isAandY_G1()){
//            double k = odometry.getGlobalPosition().getHeading()/Math.toRadians(90);
//
//            boolean ifForward = Math.abs(forwardVoltage) > Math.abs(sideVoltage);
//            boolean ifSide = Math.abs(sideVoltage) > Math.abs(forwardVoltage);
//
//            metry.getTelemetry().addData("k", k);
//
//            if(k > 0){
//                if(ifForward) {
//                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1/k);
//                } else if (ifSide) {
//                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1/k, 1);
//                }else {
//                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
//                }
//            }else if (k < 0){
//                if(ifForward) {
//                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1/k, 1);
//                } else if (ifSide) {
//                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1/k);
//                }else {
//                    drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
//                }
//            }else{
//                drivetrain.setPowerTeleOpHeadless(forwardVoltage, sideVoltage, angleVoltage, 1, 1);
//            }
//        }
    }

    // Gamepad 2
    @Override
    public synchronized void teleopPl2() {
        Gamepad g2 = joysticks.getGamepad2();

        double leftStickY = -g2.left_stick_y;

        double upStandingVel = -g2.right_stick_y;

        if(joysticks.getDpadUp(g2.dpad_down) == 1){
            horizontalPos = CLOSE_POS_HORIZONTAL - 0.1;
        } else if (joysticks.getDpadUp(g2.dpad_down) == 2) {
            horizontalPos = CLOSE_POS_HORIZONTAL - 0.2;
        }else if (joysticks.getDpadUp(g2.dpad_down) == 3) {
            horizontalPos = CLOSE_POS_HORIZONTAL - 0.3;
        }else if (joysticks.getDpadUp(g2.dpad_down) == 4) {
            horizontalPos = CLOSE_POS_HORIZONTAL - 0.4;
        }else if (joysticks.getDpadUp(g2.dpad_down) == 5) {
            horizontalPos = OPEN_POS_HORIZONTAL;
        }else {
            horizontalPos = CLOSE_POS_HORIZONTAL;
        }

        if (joysticks.isX_G2()){
            teleSkope.setTeleskopeProp(upStandingVel, horizontalPos);
        }else{
            teleSkope.setTeleskope(upStandingVel, horizontalPos);}

        if (joysticks.isB_G2()){
            teleSkope.setFlip(CONSTSTELESKOPE.HANG_POS_FLIP);
        }else {
            teleSkope.setFlip(CONSTSTELESKOPE.TAKE_POS_FLIP);
        }

        if (joysticks.isA_G2()){
            teleSkope.setHook(CONSTSTELESKOPE.OPEN_POS_HOOK);
        }else {
            teleSkope.setHook(CONSTSTELESKOPE.CLOSE_POS_HOOK);
        }

        //        if(robotAlliance.equals(RobotAlliance.RED) ){
//            if(colorSensor.getMainColor() == Colors.RED && colorSensor.getDistance() < 3){
//                closeAuto = true;
//            }
//        }
//
//        if(robotAlliance.equals(RobotAlliance.BLUE)){
//            if(colorSensor.getMainColor() == Colors.BLUE && colorSensor.getDistance() < 3){
//                closeAuto = true;
//            }
//        }
//        if(colorSensor.getMainColor() == Colors.YELLOW && colorSensor.getDistance() < 3){
//            closeAuto = true;
//        }

//    if(!joysticks.isA_G2() && closeAuto){
//        teleSkope.setHook(CLOSE_POS_HOOK);
//    }else {
//        teleSkope.setHook(OPEN_POS_HOOK);
//        }
    }

    public synchronized void initPlayersTelemetry() {
        odometry.getRobotPos();
        odometry.getEncPos();
//        drivetrain.getMotorsPower();
//        servosService.getServosPos();
//        joysticks.checkJoysticksCombo();
//        joysticks.checkGear();
//        joysticks.getDpadUp();
    }
}
