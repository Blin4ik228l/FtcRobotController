package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.RGBColorSensor;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.ServosService;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.TeleSkope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual.DataDisplayer;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Virtual.Metry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.Colors;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataFilter;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataGroup;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.DataUtils.DataTarget;
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

    // Системы робота.
    // Железо хранится уже в самой системе.
    public final Odometry odometry; // Система вычислений одометрии
    public final MecanumDrivetrain drivetrain; // Телега робота
    public final TeleSkope teleSkope;
    public final Metry metry;
    public final Joysticks joysticks;
    public final DataDisplayer dataDisplayer;
    public final ServosService servosService;
    public final RGBColorSensor colorSensor;

    double horizontalPos = CLOSE_POS_HORIZONTAL, forwardC, sideC;

    float gain = 2;
    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;
    public final float[] hsvValues = new float[3];
    // ПИД объекты должны быть final, инициализироваться здесь,
    // либо извне через PID.setPID(ваши коэффициенты)
    public final PID pidLinearX = new PID(0.018,0.00000022,0.0000, -1,1);
    public final PID pidLinearY = new PID(0.018,0.00000022,0.0000, -1,1);
    public final PID pidAngular = new PID(0.95,0.000018,0, -1,1);

////////////////////////////////////////////////////////////////////////////////////////////////////

    public Robot(RobotMode robotMode, RobotAlliance robotAlliance, OpMode op) {
        super(robotMode, robotAlliance, op);

        metry = new Metry(op);
        joysticks = new Joysticks(op);
        odometry = new Odometry(op);
        drivetrain = new MecanumDrivetrain(op);
        servosService = new ServosService(op);
        teleSkope = new TeleSkope(op, servosService);
        colorSensor = new RGBColorSensor(op);

        dataDisplayer = new DataDisplayer(this);
    }
    @Override
    // Метод инициализации того, чего надо
    public void init() {
        odometry.init();
        drivetrain.init();
        servosService.init();
        teleSkope.init();
        joysticks.init();
        metry.init();
        colorSensor.init();

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
                dataDisplayer.addData("result", result);

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
            StandartArgs.teleskopeStandartArgs args = (StandartArgs.teleskopeStandartArgs) _args;
            if(args.teleskope_height > 106){
                ((StandartArgs.teleskopeStandartArgs) _args).teleskope_height = 106;
            }
            return 0;
        }
        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
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

            dataDisplayer.addData("resultTele", result);
            dataDisplayer.showGroupData(DataGroup.TELESKOPE, DataTarget.displayCurHeightTeleskope,DataFilter.CM);
            dataDisplayer.update();
            // TODO: обработчик застреваний телескопа
            //  если робот вдруг поехал
            //  если телескоп не поднялся на нужный уровень и стоит на месте долго

            return result;
        }
    };

    public synchronized void checkJoysticks(){
        joysticks.checkJoysticksCombo();
    }
    public synchronized void updateColors(){colorSensor.update();}

    public synchronized void telemetry(){
        if(robotMode == RobotMode.TELEOP){
            dataDisplayer.dataForTeleOp();
        }else if(robotMode == RobotMode.AUTO){
            dataDisplayer.dataForAuto();
        }
    }
    public synchronized void runTeleopMethods(){

        class runTele1 implements Runnable{
            private Thread c1;

            @Override
            public void run() {
                try {
                teleopPl1();
                } catch (Exception e) {
                    dataDisplayer.addLine("Calc thread interrupted tele1");
                    dataDisplayer.update();
                }
            }
            public  void start_c(){
                if(c1 == null){
                    c1 = new Thread(this, "Calc Tele1");
                    c1.start();
                }
            }
            public void setDaemon(boolean bol){
                c1.setDaemon(bol);
            }
        }

        class runTele2 implements Runnable{
            private Thread c2;

            @Override
            public void run() {
                try {
                    teleopPl2();
                } catch (Exception e) {
                    dataDisplayer.addLine("Calc thread interrupted tele2");
                    dataDisplayer.update();
                }
            }

            public  void start_c(){
                if(c2 == null){
                    c2 = new Thread(this, "Calc Tele2");
                    c2.start();
                }
            }
            public void setDaemon(boolean bol){
                c2.setDaemon(bol);
            }
        }

        runTele2 Tele2 = new runTele2();
        runTele1 Tele1 = new runTele1();

        Tele1.setDaemon(true);
        Tele1.start_c();

        Tele2.setDaemon(true);
        Tele2.start_c();
    }

    // Gamepad 1
    @Override
    public synchronized void teleopPl1() {
        Gamepad g1 = joysticks.getGamepad1();

        double max_speed = 0.6;
        double accelLinear, accelAngle;

        if(g1.left_trigger > 0.05){
            accelLinear = 1.5;
        }else {accelLinear = 1;}

        if(g1.right_trigger > 0.05){
            accelAngle = 1.5;
        }else{
            accelAngle = 1;
        }

        double forwardVoltage = Range.clip(-g1.left_stick_y , -max_speed* accelLinear, max_speed* accelLinear);
        double sideVoltage = Range.clip(g1.left_stick_x ,  -max_speed* accelLinear, max_speed* accelLinear);
        double angleVoltage = Range.clip(g1.right_stick_x , -max_speed* accelAngle, max_speed* accelAngle);

        if(joysticks.isHeadlessDrive()){
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

        if(joysticks.isCruiseDrive()){
            forwardC = Range.clip(forwardC + (-Range.clip(g1.left_stick_y,-0.02, 0.02)), -0.4, 0.4);
            sideC = Range.clip(sideC + (Range.clip(g1.left_stick_y,-0.02, 0.02)), -0.4, 0.4);

            drivetrain.setPowerTeleOp(forwardC, sideC, angleVoltage);
        }else {
            drivetrain.setPowerTeleOp(forwardVoltage, sideVoltage, angleVoltage);
        }

//        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.ENCL, DataFilter.CM);
//        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.ENCR, DataFilter.CM);
//        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.ENCM, DataFilter.CM);

//        dataDisplayer.addData("forwardVoltage", forwardVoltage);
//        dataDisplayer.addData("sideVoltage", sideVoltage);
//        dataDisplayer.addData("angleVoltage", angleVoltage);
//
//        dataDisplayer.addData("isCruise", joysticks.isCruiseDrive());
    }

    // Gamepad 2
    @Override
    public synchronized void teleopPl2() {
        Gamepad g2 = joysticks.getGamepad2();
        boolean closeAuto = false;

        double upStandingVel = -g2.right_stick_y;

        horizontalPos = Range.clip(horizontalPos + (-Range.clip(g2.left_stick_y,-0.4, 0.4)), OPEN_POS_HORIZONTAL,CLOSE_POS_HORIZONTAL);

        if(robotAlliance.equals(RobotAlliance.RED) ){
            if(colorSensor.getMainColor() == Colors.RED && colorSensor.getDistance() < 3){
                closeAuto = true;
            }
        }

        if(robotAlliance.equals(RobotAlliance.BLUE)){
            if(colorSensor.getMainColor() == Colors.BLUE && colorSensor.getDistance() < 3){
                closeAuto = true;
            }
        }
        if(colorSensor.getMainColor() == Colors.YELLOW && colorSensor.getDistance() < 3){
            closeAuto = true;
        }

    if(closeAuto && !joysticks.isHookOpen()){
        teleSkope.setHook(CLOSE_POS_HOOK);

    }else {
        teleSkope.setHook(OPEN_POS_HOOK);
        }


        if (joysticks.isProportionalTeleskope()){
            teleSkope.setTeleskopeProp(upStandingVel, horizontalPos);
        }else{
            teleSkope.setTeleskope(upStandingVel, horizontalPos);}

//        metry.getTelemetry().addLine()
//                .addData("Red", colorSensor.getRed())
//                .addData("Green", colorSensor.getGreen())
//                .addData("Blue", colorSensor.getBlue());
//        metry.getTelemetry().addLine()
//                .addData("Distance", colorSensor.getDistance())
//                .addData("MainColor", colorSensor.getMainColor().toString());


//        dataDisplayer.addData("isProp", joysticks.isProportionalTeleskope());
//        dataDisplayer.addData("isHookOpen", joysticks.isHookOpen());
//        dataDisplayer.addData("hookPos", servosService.getHook().getPosition());
//        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.ENCL, DataFilter.CM);
//        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.UPSTANDINGLEFT, DataFilter.CM);
//        dataDisplayer.showValue(DataTarget.displayCurPosition, DataObject.UPSTANDINGRIGHT, DataFilter.CM);
//        dataDisplayer.showValue(DataTarget.displayOtherPosition, DataObject.HORIZONTAL, DataFilter.POSITION);
//
//        dataDisplayer.addData("horizontalPos", horizontalPos);
//        dataDisplayer.showValueJoystick(DataTarget.displayJoystickStateMent, DataObject.GAMEPAD2, JoystickStatement.LEFT_STICK);
    }
}
