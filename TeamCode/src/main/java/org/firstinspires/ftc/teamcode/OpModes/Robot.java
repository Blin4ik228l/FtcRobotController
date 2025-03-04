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
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.ComonStatuses.EncoderStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.ComonStatuses.MotorsStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.OtherStatuses.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.OtherStatuses.TeleskopeStatusInMoving;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.RobotModuleStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers.DriveHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers.TeleskopeHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers.ZahvatHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.PID;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

import java.util.ArrayDeque;
import java.util.Deque;

public class Robot extends RobotCore implements Consts, ConstsTeleskope {
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
    public final Button button;

    public RobotModuleStatus robotStatus;
    public RobotModuleStatus teleskopeStatus;
//    public final RGBColorSensor colorSensor;
//    public final Distance distanceSensor;
//    public final TaskManager taskManager;

    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)

    public final PID pidLinearX = new PID(0.018,0.0000001,0.000, -1,1);
    public final PID pidLinearY = new PID(0.018,0.0000001,0.000, -1,1);
    public final PID pidAngular = new PID(0.27,0.0,0.000, -1,1);

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

        this.robotStatusHandler.robot = this;

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

        robotStatus = RobotModuleStatus.Normal;
        teleskopeStatus = RobotModuleStatus.Normal;
//        colorSensor.init();
//        distanceSensor.init();
    }



    // Метод, обрабатывающий задачу перемещения робота в точку
    public DriveHandler driveToPosition = new DriveHandler() {
        //Сделать статусы состояний робота в разных действиях, для дальнейших работ с ним
       public RobotModuleStatus statusInDrive;

       public RobotStatusInDrive statusBy_X;
       public RobotStatusInDrive statusBy_Y;
       public RobotStatusInDrive statusBy_Rotate;

       public final Deque<RobotStatusInDrive> statusBy_X_history = new ArrayDeque<>();
       public final Deque<RobotStatusInDrive> statusBy_Y_history = new ArrayDeque<>();
       public final Deque<RobotStatusInDrive> statusBy_Rotate_history = new ArrayDeque<>();

       public MotorsStatus motorsStatus;

       public final ElapsedTime stuckTimeBy_X = new ElapsedTime();
       public final ElapsedTime stuckTimeBy_Y = new ElapsedTime();
       public final ElapsedTime stuckTimeBy_Rotate = new ElapsedTime();

       public double progressBar;

        @Override
        public Deque[] statusInDrive() {
            return new Deque[]{statusBy_X_history, statusBy_Y_history, statusBy_Rotate_history};
        }

        @Override
        public RobotModuleStatus statusRobot() {
            return statusInDrive;
        }

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.driveArgs args = (StandartArgs.driveArgs) _args;

            if(statusBy_X == null && statusBy_Y == null && statusBy_Rotate == null) {
                statusBy_X = args.position.getX() == 0 ? RobotStatusInDrive.NoneBy_X: RobotStatusInDrive.StayingBy_X;
                statusBy_Y = args.position.getY() == 0 ? RobotStatusInDrive.NoneBy_Y: RobotStatusInDrive.StayingBy_Y;
                statusBy_Rotate = args.position.getHeading() == 0 ? RobotStatusInDrive.NoneBy_Rotate: RobotStatusInDrive.StayingBy_Rotate;

                statusBy_X_history.addLast(statusBy_X);
                statusBy_Y_history.addLast(statusBy_Y);
                statusBy_Rotate_history.addLast(statusBy_Rotate);
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


            /**Если перемещение по энкодерам не велико и на моторы подаётся напряжение, то начинается отсчёт времени застревания
             * Оно нужно для того, чтобы робот возможно успел бы вернутся на траекторию движения, например:
             * Если бы робот ударился и начал ехать в другого робота, но тот бы отъехал и наш бы продолжил движение
             * Если же нет, то тогда считаем что робот застрял, въехал в стенку или игровую конструкцию
             */

            if (
                    (odometry.getEncXst() == EncoderStatus.SmallDelta
                            || odometry.getEncXst() == EncoderStatus.ZeroDelta)
                            && motorsStatus == MotorsStatus.Powered && errorPos.x != 0
            )stuckTimeBy_X.seconds();
            else stuckTimeBy_X.reset();

            if (
                    (odometry.getEncYst() == EncoderStatus.SmallDelta
                            || odometry.getEncYst() == EncoderStatus.ZeroDelta)
                            && motorsStatus == MotorsStatus.Powered && errorPos.y != 0
            )stuckTimeBy_Y.seconds();
            else stuckTimeBy_Y.reset();

            if (
                    (odometry.getEncRadSt() == EncoderStatus.SmallDelta
                            || odometry.getEncRadSt() == EncoderStatus.ZeroDelta)
                            && motorsStatus == MotorsStatus.Powered && errorHeading != 0
            )stuckTimeBy_Rotate.seconds();
            else stuckTimeBy_Rotate.reset();


            if(stuckTimeBy_X.seconds() > 2){
                statusBy_X = RobotStatusInDrive.StuckedBy_X;//Застревание при движении по X
                if(statusBy_X_history.getLast() != statusBy_X)statusBy_X_history.addLast(statusBy_X);
            }
            if(stuckTimeBy_Y.seconds() > 2){
                statusBy_Y = RobotStatusInDrive.StuckedBy_Y;//Застревание при движении по Y
                if(statusBy_Y_history.getLast() != statusBy_Y)statusBy_Y_history.addLast(statusBy_Y);
            }
            if(stuckTimeBy_Rotate.seconds() > 2){
                statusBy_Rotate = RobotStatusInDrive.StuckedBy_Rotate;//Застревание при повороте
                if(statusBy_Rotate_history.getLast() != statusBy_Rotate)statusBy_Rotate_history.addLast(statusBy_Rotate);
            }

            statusInDrive = statusBy_Rotate == RobotStatusInDrive.StuckedBy_Rotate
                    || statusBy_Y == RobotStatusInDrive.StuckedBy_Y
                    ||statusBy_X == RobotStatusInDrive.StuckedBy_X
                    ? RobotModuleStatus.Stucked:RobotModuleStatus.Moving;

            // Направление движения
            Vector2 targetVel = new Vector2(errorPos);
            targetVel.normalize();
            targetVel.rotate(-odometry.getGlobalPosition().getHeading()); // Здесь минус потому что направление движения поворачивается в обратную сторону относительно поворота робота!!!

            // Выбираем скорости в зависимости от величины ошибки
            linearVel = errorPos.length() > returnDistance(args.max_linear_speed, MAX_LINEAR_ACCEL) ? args.max_linear_speed : MIN_LINEAR_SPEED;

            if(Math.abs(targetVel.y) > MAX_LINEAR_SIDE){
                targetVel.multyplie(MAX_LINEAR_SIDE/Math.abs(targetVel.y));
            }

            if(linearVel < MIN_LINEAR_SPEED) linearVel = MIN_LINEAR_SPEED;// Ограничиваем скорость снизу

            if (errorPos.length() < 2){
                errorPosDone = true;
                linearVel = 0;
            }

            if(Math.abs(errorHeading) < Math.toRadians(2) ){
                errorHeadingDone = true;

            }
            targetVel.multyplie(linearVel);

            // Передаем требуемые скорости в ПИД для расчета напряжения на моторы
            double speedPIDX = pidLinearX.calculate(targetVel.x, odometry.getVelocity().x);
            double speedPIDY = pidLinearY.calculate(targetVel.y, odometry.getVelocity().y);
            double angularPID = pidAngular.calculate(args.position.getHeading(), odometry.getGlobalPosition().getHeading());

            if(errorPosDone && errorHeadingDone){
                result = 0;
                motorsStatus = drivetrain.offMotors();

                statusInDrive = RobotModuleStatus.Normal;

                if(statusBy_X != RobotStatusInDrive.StuckedBy_X){
                    statusBy_X = RobotStatusInDrive.CompletedBy_X;
                    statusBy_X_history.addLast(statusBy_X);
                }
                if(statusBy_Y != RobotStatusInDrive.StuckedBy_Y){
                    statusBy_Y = RobotStatusInDrive.CompletedBy_Y;
                    statusBy_Y_history.addLast(statusBy_Y);
                }
                if(statusBy_Rotate != RobotStatusInDrive.StuckedBy_Rotate){
                    statusBy_Rotate = RobotStatusInDrive.CompletedBy_Rotate;
                    statusBy_Rotate_history.addLast(statusBy_Rotate);
                }
            }else{
                result = -1;

                statusInDrive = RobotModuleStatus.Moving;


                //Проверяем на отклонение от курса
                if(errorPos.x > args.position.getX() && statusBy_X != RobotStatusInDrive.NoneBy_X){
                    statusBy_X = RobotStatusInDrive.MovingInOtherSideBy_X;
                    if(statusBy_X_history.getLast() != statusBy_X)statusBy_X_history.addLast(statusBy_X);
                }else{
                    statusBy_X = RobotStatusInDrive.MovingBy_X;
                    if(statusBy_X_history.getLast() != statusBy_X)statusBy_X_history.addLast(statusBy_X);
                }
                if(errorPos.y > args.position.getX() && statusBy_Y != RobotStatusInDrive.NoneBy_Y){
                    statusBy_Y = RobotStatusInDrive.MovingInOtherSideBy_Y;
                    if(statusBy_Y_history.getLast() != statusBy_Y)statusBy_Y_history.addLast(statusBy_Y);
                }else {
                    statusBy_Y = RobotStatusInDrive.MovingBy_Y;
                    if(statusBy_Y_history.getLast() != statusBy_Y) statusBy_Y_history.addLast(statusBy_Y);
                }

                motorsStatus = drivetrain.setXYHeadVel(speedPIDX, speedPIDY, angularPID);
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
    public TeleskopeHandler setTeleskopePos = new TeleskopeHandler() {

        MotorsStatus motorsStatus;

        RobotModuleStatus teleskopeStatus;
        TeleskopeStatusInMoving teleskopeStatusInMoving;
        Deque<TeleskopeStatusInMoving> teleskopeStatusHistory = new ArrayDeque<>();

        ElapsedTime stuckTime;

        @Override
        public Deque<TeleskopeStatusInMoving>[] status() {
            return new Deque[]{teleskopeStatusHistory};
        }

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.teleskopeArgs args = (StandartArgs.teleskopeArgs) _args;
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
            StandartArgs.teleskopeArgs args = (StandartArgs.teleskopeArgs) _args;

            if(teleskopeStatus == null){
                teleskopeStatusInMoving = args.teleskope_height == 0 ? TeleskopeStatusInMoving.None : TeleskopeStatusInMoving.Staying;
                teleskopeStatusHistory.addLast(teleskopeStatusInMoving);
            }

            int result;

            double target = args.teleskope_height - teleSkope.getHeight();
            boolean horizontalPosDone = false;

            if ((teleSkope.leftEncUpSt == EncoderStatus.ZeroDelta || teleSkope.leftEncUpSt == EncoderStatus.SmallDelta)
            && (teleSkope.rightEncUpSt == EncoderStatus.ZeroDelta || teleSkope.rightEncUpSt == EncoderStatus.SmallDelta)
            && motorsStatus == MotorsStatus.Powered && !button.isTouched()) stuckTime.seconds();
            else stuckTime.reset();

            if(stuckTime.seconds() > 1){
                if(args.teleskope_height > teleSkope.getHeight()){
                    teleskopeStatusInMoving = TeleskopeStatusInMoving.StuckedBy_Upping;
                    teleskopeStatusHistory.addLast(teleskopeStatusInMoving);
                }

                if(args.teleskope_height < teleSkope.getHeight()){
                    teleskopeStatusInMoving = TeleskopeStatusInMoving.StuckedBy_Downing;
                    teleskopeStatusHistory.addLast(teleskopeStatusInMoving);
                }
            }

            if(teleskopeStatusInMoving == TeleskopeStatusInMoving.StuckedBy_Downing || teleskopeStatusInMoving == TeleskopeStatusInMoving.StuckedBy_Upping){
                teleskopeStatus = RobotModuleStatus.Stucked;
            }else teleskopeStatus = RobotModuleStatus.Normal;

            if(servosService.getHorizontal().getPosition() != args.servo_pos){

                teleSkope.setPosHorizontalTeleOp(args.servo_pos);

                horizontalPosDone = true;
            }

            if(Math.abs(target) < 1.8 && horizontalPosDone){
                result = 0;

                motorsStatus = teleSkope.offMotors();

                teleskopeStatusInMoving = TeleskopeStatusInMoving.Completed;
                teleskopeStatusHistory.addLast(teleskopeStatusInMoving);
            }else {
                result = -1;

                if(args.teleskope_height > teleSkope.getHeight()){
                    teleskopeStatusInMoving = TeleskopeStatusInMoving.Upping;
                    teleskopeStatusHistory.addLast(teleskopeStatusInMoving);
                }

                if(args.teleskope_height < teleSkope.getHeight()){
                    teleskopeStatusInMoving = TeleskopeStatusInMoving.Downing;
                    teleskopeStatusHistory.addLast(teleskopeStatusInMoving);
                }

                motorsStatus = teleSkope.setTeleskopeAuto(args.max_speed, args.teleskope_height);
            }

            op.telemetry.addData("Teleskope height", teleSkope.getHeight());
            op.telemetry.addData("Math.abs(target) < 3", Math.abs(target) < 3);
            op.telemetry.addData("horizontalPosDone", horizontalPosDone);
            op.telemetry.addData("result", result);
            op.telemetry.update();

            return result;
        }
    };

    public ZahvatHandler setZahvat = new ZahvatHandler(){

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.captureArgs args = (StandartArgs.captureArgs) _args;
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

    public TaskHandlerOrdinal robotSleep = new TaskHandlerOrdinal() {

        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {

            StandartArgs.robotSleep args = (StandartArgs.robotSleep) _args;

            ElapsedTime runTime = new ElapsedTime();

            int result = -1;

            while (runTime.milliseconds() < args.time){
                result = 0;
            }

            return result;
        }

    };

    public TaskHandlerOrdinal doWhile = new TaskHandlerOrdinal() {
        @Override
        public int init(TaskManager thisTaskManager, StandartArgs _args) {
            return 0;
        }

        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.doWhile args = (StandartArgs.doWhile) _args;
            int result;

            if (!button.isTouched() && button.getTimesTouched() != 1){
                teleSkope.setTeleskope(args.power, ConstsTeleskope.CLOSE_POS_HORIZONTAL);
                result = -1;
            }else{
                teleSkope.init();
                result = 0;
            }

            return result;
        }

    };

    // Gamepad 1
    @Override
    public synchronized void teleopPl1() {
        Gamepad g1 = joysticks.getGamepad1();

        double max_speed = 0.5;
        double oldmax_speed = max_speed;
        double accelLinear = 1.3, accelAngle = 1.3;

        if(g1.left_trigger > 0.05 && g1.right_trigger < 0.05){//Ускорение робота
            accelLinear = 1.8;
            accelAngle = 1.8;
        }

        if(g1.right_trigger > 0.05 && g1.left_trigger < 0.05){//Замедление робота
            accelLinear = 0.25;
            accelAngle = 0.25;
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

        op.telemetry.addData("Max_speed", max_speed);// выводим максимальную скорость
        op.telemetry.addData("Gear", joysticks.getGear());// выводим передачу
        op.telemetry.addData("Is up gear", joysticks.isUpGear());
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

        if(servosService.getHook().getPosition() == CLOSE_POS_HOOK){
            drivetrain.onLed();
        }
        if(servosService.getHook().getPosition() == OPEN_POS_HOOK){
            drivetrain.offLed();
        }

        if (joysticks.isX_G2()){
            teleSkope.setTeleskopeProp(upStandingVel, horizontalPos);
        }else{
            teleSkope.setTeleskope(upStandingVel, horizontalPos);}

        if (joysticks.isB_G2()){
            teleSkope.setFlip(ConstsTeleskope.HANG_POS_FLIP);
        }else {
            teleSkope.setFlip(ConstsTeleskope.TAKE_POS_FLIP);
        }

        if (joysticks.isA_G2()){
            teleSkope.setHook(ConstsTeleskope.OPEN_POS_HOOK);
        }else {
            teleSkope.setHook(ConstsTeleskope.CLOSE_POS_HOOK);
        }
        if (joysticks.isY_g1()){
            init();
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
