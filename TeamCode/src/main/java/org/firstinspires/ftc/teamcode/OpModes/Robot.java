package org.firstinspires.ftc.teamcode.OpModes;

import android.app.Activity;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.ColorSensor;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Joysticks;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Odometry;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.ServosService;
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
    public final ColorSensor colorSensor;

    double horizontalPos = CLOSE_POS_HORIZONTAL, forwardC, sideC;

    float gain = 2;
    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;
    final float[] hsvValues = new float[3];

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
        servosService = new ServosService(op);
        teleSkope = new TeleSkope(op, servosService);
        colorSensor = new ColorSensor(op);

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
            if(args.teleskope_height > 120){
                ((StandartArgs.teleskopeStandartArgs) _args).teleskope_height = 120;
            }
            return 0;
        }
        @Override
        public int execute(TaskManager thisTaskManager, StandartArgs _args) {
            StandartArgs.teleskopeStandartArgs args = (StandartArgs.teleskopeStandartArgs) _args;
            int result;

            double target = args.teleskope_height - teleSkope.getHeight();

            if(target < 1){
                teleSkope.offMotors();
                result = 0;
            }else {
                teleSkope.setTeleskopePropAuto(args.max_speed, args.servo_pos, args.teleskope_height);
                result = -1;
            }


            // TODO: обработчик застреваний телескопа
            //  если робот вдруг поехал
            //  если телескоп не поднялся на нужный уровень и стоит на месте долго

            return result;
        }
    };

    public synchronized void checkJoysticks(){
        joysticks.checkJoysticksCombo();
    }

    public synchronized void telemetry(){
        if(robotMode == RobotMode.TELEOP){
            dataDisplayer.dataForTeleOp();
        }else if(robotMode == RobotMode.AUTO){
            dataDisplayer.dataForAuto();
        }
    }

    // Gamepad 1
    @Override
    public synchronized void teleopPl1() {
        Gamepad g1 = joysticks.getGamepad1();



        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).


        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.


        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad


        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }


            // Explain basic gain information via telemetry
        metry.getTelemetry().addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        metry.getTelemetry().addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

            // Update the gain value if either of the A or B gamepad buttons is being held
            if (g1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (g1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }

            // Show the gain value via telemetry
        dataDisplayer.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensor.getSensorColor().setGain(gain);

            // Check the status of the X button on the gamepad
            xButtonCurrentlyPressed = g1.x;

            // If the button state is different than what it was, then act
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                // If the button is (now) down, then toggle the light
                if (xButtonCurrentlyPressed) {
                    if (colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight)colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getSensorColor().getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);

        metry.getTelemetry().addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
        metry.getTelemetry().addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
        metry.getTelemetry().addData("Alpha", "%.3f", colors.alpha);

            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity. */
            if (colorSensor instanceof DistanceSensor) {
                metry.getTelemetry().addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }





//        double max_speed = 0.6;
//        double accelLinear, accelAngle;
//
//        if(g1.left_trigger > 0.05){
//            accelLinear = 1.5;
//        }else {accelLinear = 1;}
//
//        if(g1.right_trigger > 0.05){
//            accelAngle = 1.5;
//        }else{
//            accelAngle = 1;
//        }
//
//        double forwardVoltage = Range.clip(-g1.left_stick_y , -max_speed* accelLinear, max_speed* accelLinear);
//        double sideVoltage = Range.clip(g1.left_stick_x ,  -max_speed* accelLinear, max_speed* accelLinear);
//        double angleVoltage = Range.clip(g1.right_stick_x , -max_speed* accelAngle, max_speed* accelAngle);
//
//        if(joysticks.isHeadlessDrive()){
//            double k = odometry.getGlobalPosition().getHeading()/Math.toRadians(90);
//
//            boolean ifForward = Math.abs(forwardVoltage) > Math.abs(sideVoltage);
//            boolean ifSide = Math.abs(sideVoltage) > Math.abs(forwardVoltage);
//
//            dataDisplayer.addData("k", k);
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
//
//        if(joysticks.isCruiseDrive()){
//            forwardC = Range.clip(forwardC + (-g1.left_stick_y/15), -0.4, 0.4);
//            sideC = Range.clip(sideC + (g1.left_stick_x/15), -0.4, 0.4);
//
//            drivetrain.setPowerTeleOp(forwardC, sideC, angleVoltage);
//        }else {
//            drivetrain.setPowerTeleOp(forwardVoltage, sideVoltage, angleVoltage);
//        }
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

        double upStandingVel = -g2.right_stick_y;

        horizontalPos = Range.clip(horizontalPos + (-g2.left_stick_y/18), OPEN_POS_HORIZONTAL,CLOSE_POS_HORIZONTAL);

        teleSkope.setHook(joysticks.isHookOpen());

        if (joysticks.isProportionalTeleskope()){
            teleSkope.setTeleskopeProp(upStandingVel, horizontalPos);
        }else{
            teleSkope.setTeleskope(upStandingVel, horizontalPos);}

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
