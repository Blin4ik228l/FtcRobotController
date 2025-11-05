package org.firstinspires.ftc.teamcode.Robot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Robot.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.TeamColor;

import java.util.HashMap;

public class RobotClass extends TeamColor {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */

    public RobotClass(OpMode op, String teamColor){
        super(teamColor);

        driveTrain = new MecanumDrivetrain(op, this);
        collector = new Collector(op);
    }
    public static MecanumDrivetrain driveTrain;
    public static Collector collector;
    public static class MecanumDrivetrain extends Module {
        //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.

        public MecanumDrivetrain(OpMode op, TeamColor teamColor){
            super(op.telemetry);

            rightB = op.hardwareMap.get(DcMotor.class, "rightB");
            rightF = op.hardwareMap.get(DcMotor.class, "rightF");
            leftB = op.hardwareMap.get(DcMotor.class, "leftB");
            leftF = op.hardwareMap.get(DcMotor.class, "leftF");

            rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightB.setDirection(DcMotorSimple.Direction.FORWARD);
            rightF.setDirection(DcMotorSimple.Direction.FORWARD);
            leftB.setDirection(DcMotorSimple.Direction.REVERSE);
            leftF.setDirection(DcMotorSimple.Direction.REVERSE);

            rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            exOdometry = new ExOdometry(op,teamColor );

            telemetry.addLine("Drivetrain Inited");
        }
        public DcMotor leftB;
        public DcMotor leftF;
        public DcMotor rightB;
        public DcMotor rightF;
        public ExOdometry exOdometry;
        public void setPower(double yVol, double xVol, double angVol){
            if(Math.abs(yVol) < 0.10) yVol = 0.10 * Math.signum(yVol);
            if(Math.abs(xVol) < 0.10) xVol = 0.10 * Math.signum(xVol);
            if(Math.abs(angVol) < 0.10) angVol = 0.10 * Math.signum(angVol);

            yVol *= 1;
            xVol *= 1.1;
            angVol *= 1.3;

            //движение по y - это вперёд - назад
            //движение по x - это влево - вправо
            //angVol поворот
            exOdometry.updateAll();//Обноволяем одометрию постоянно когда вызываем метод setPower

            leftF.setPower( yVol - xVol - angVol);
            leftB.setPower( yVol + xVol - angVol);
            rightF.setPower(yVol + xVol + angVol);
            rightB.setPower(yVol - xVol + angVol);
        }
    }

    public static class Collector extends Module{
        public Collector(OpMode op) {
            super(op.telemetry);
            motors = new Motors(op);
            servos = new Servos(op);
            colorSensor = new ColorSensor(op);

            telemetry.addLine("Teleskope inited");
        }
        public Motors motors;
        public Servos servos;
        public ColorSensor colorSensor;
        public void startLoading(boolean isTurnOn){
            if(isTurnOn) {
                if(servos.selfData.valueOfCell.size() == 3) motors.turnOnFlyWheel(true);
                else motors.turnOnInTake(true);

                colorSensor.update();
                servos.barabanNextPos(colorSensor.currentArtifact);
            }

//            servos.setBaraban(sPosBaraban);
//            servos.setAngle(sPosUlitka);
//            servos.setPusher(sPosRamp);
        }

        public void startFiring(){

        }
        public class Motors {
            public Motors(OpMode op){
                selfData = new SelfData();

                inTake = op.hardwareMap.get(DcMotor.class, "inTake");
                flyWheelRight = op.hardwareMap.get(DcMotor.class, "flyWheelRight");
                flyWheelLeft = op.hardwareMap.get(DcMotor.class, "flyWheelLeft");

                inTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flyWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flyWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//                inTake.setDirection(DcMotorSimple.Direction.FORWARD);
//                flyWheelRight.setDirection(DcMotorSimple.Direction.REVERSE);

                inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flyWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                flyWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                telemetry.addLine("Motors on collector inited");
            }
            private final DcMotor inTake;
            private final DcMotor flyWheelRight;
            private final DcMotor flyWheelLeft;
            public final SelfData selfData;

            public void turnOnInTake(boolean bol){
                if(bol) inTake.setPower(1);
                else inTake.setPower(0);
            }
            public void turnOnFlyWheel(boolean bol){
                if(bol){
                    flyWheelLeft.setPower(-1);
                    flyWheelRight.setPower(1);
                }else{
                    flyWheelLeft.setPower(0);
                    flyWheelRight.setPower(0);
                }
            }
            public void setSpeedOnFlyWheel(double speed){

            }



            public class SelfData{

            }
            double getAngle(double length){
                double velMahovik = (6000 * 8 * Math.PI) / 60;

                double velBall = 2500;
                //580

                return ((length * 981) / (Math.pow(velBall, 2)));
            }

            double getLength(double angle){
                double velMahovik = (6000 * 8 * Math.PI) / 60;

                double velBall = 2500;

                return (Math.pow(velBall, 2) * Math.sin(angle * 2)) / 981;
            }
        }
        public class Servos{
            public Servos(OpMode op){
                pusher = op.hardwareMap.get(Servo.class, "pusher");
                baraban = op.hardwareMap.get(Servo.class, "baraban");
                angle = op.hardwareMap.get(Servo.class, "angle");

                //Устанавливаем в начальное положение
                setPusher(PUSHER_START_POS);
                setBaraban(BARABAN_START_POS);
                setAngle(ANGLE_START_POS);

                selfData = new SelfData();

                telemetry.addLine("Servos inited");
            }
            public class SelfData{
                public SelfData(){
                    loadedArtifacts = new HashMap<>();
                    valueOfCell = new HashMap<>();
                }
                HashMap<Integer, HashMap<Integer, Double>> loadedArtifacts;
                HashMap<Integer, Double> valueOfCell;

                int currentArtifact;
                double barabanNextPos;

                double barabanTargetPos;
                int numOfCell = 0;

                int count2;
                int count;
                boolean isRobotFiring = false;
                public void calculate(){
                    if(currentArtifact != 0 && loadedArtifacts.size() != 3 && !isRobotFiring) {

                        valueOfCell.put(currentArtifact, baraban.getPosition());
                        loadedArtifacts.put(numOfCell, valueOfCell);

                        numOfCell += 1;
                        currentArtifact = 0;
                        barabanNextPos += 0.22;

                        count2 = numOfCell;
                    }
                    if(isRobotFiring){
                        if(loadedArtifacts.get(count2).get(driveTrain.exOdometry.camera.randomizedArtifact[0]) != null){
                            barabanTargetPos = loadedArtifacts.get(count2).get(driveTrain.exOdometry.camera.randomizedArtifact[0]);
                        }
                        count -= 1;

                       
                    }
                }

                public void setArtifact(int color){
                    currentArtifact = color;
                }
            }
            private final SelfData selfData;
            private final Servo angle;
            private final Servo pusher;
            private final Servo baraban;

            public Servo getBaraban() {
                return baraban;
            }

            public Servo getPusher() {
                return pusher;
            }

            public Servo getAngle() {
                return angle;
            }

            private void setAngle(double pos){
                angle.setPosition(pos);
            }
            private void setPusher(double pos){
                pusher.setPosition(pos);
            }
            private void setBaraban(double pos){
                baraban.setPosition(pos);
            }

            public void barabanNextPos(int color){
                selfData.setArtifact(color);
                selfData.calculate();
                baraban.setPosition(selfData.barabanNextPos);
            }

            public void findNeededArtifact(){
                selfData.calculate();
            }

            public void showServosPos(){
                telemetry.addLine("Servos statements")
                        .addData("\nzahvat", angle.getPosition())
                        .addData("\nhook", pusher.getPosition())
                        .addData("\nrotate", baraban.getPosition());
                telemetry.addLine();
            }
        }

        public class ColorSensor{
            public ColorSensor(OpMode op){
                colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

                colorSensor.setGain(gain);

                relativeLayoutId = op.hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", op.hardwareMap.appContext.getPackageName());
                relativeLayout = ((Activity) op.hardwareMap.appContext).findViewById(relativeLayoutId);

                telemetry.addLine("ColorSensor Inited");
            }
            private final int relativeLayoutId;
            private final NormalizedColorSensor colorSensor;
            private final View relativeLayout;
            private float gain = 2f;
            private final float[] hsvValues = new float[3];
            public NormalizedRGBA colors;

            public float red;
            public float blue;
            public float green;
            public float alpha;

            public int currentArtifact;

            public void update(){
                colors = colorSensor.getNormalizedColors();

                Color.colorToHSV(colors.toColor(), hsvValues);

                red = colors.red;
                blue = colors.blue;
                green = colors.green;
                alpha = colors.alpha;

                if(getDominantColor() == green) currentArtifact = 1;
                if(getDominantColor() == blue || getDominantColor() == red) currentArtifact = 2;
            }

            public float getDominantColor(){
                if(red > blue && red > green) return red;
                if(blue > red && blue > green) return blue;
                if(green > red && green > blue) return green;

                return 0;
            }

            public void showData(){
                telemetry.addLine("Values from sensor")
                        .addData("Red", "%.3f", red)
                        .addData("Green", "%.3f", green)
                        .addData("Blue", "%.3f", blue);
                telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValues[0])
                        .addData("Saturation", "%.3f", hsvValues[1])
                        .addData("Value", "%.3f", hsvValues[2]);
                telemetry.addData("Alpha", "%.3f", alpha);
            }
        }

    }
}

