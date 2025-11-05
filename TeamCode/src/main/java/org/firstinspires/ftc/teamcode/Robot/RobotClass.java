package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Robot.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.TeamColor;

public class RobotClass extends TeamColor {
   /* Основная идея данного класса:
    Робот - это как конструктор, он состоит из разных частей
    Моя задача как программиста расписать каждый модуль(часть робота)
    Чтобы в дальнейшем ими легко управлять
    */

    public RobotClass(OpMode op, String teamColor){
        super(teamColor);

//        driveTrain = new MecanumDrivetrain(op, this);
        collector = new Collector(op);// Пока не на роботе
    }
    public MecanumDrivetrain driveTrain;
    public Collector collector;
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

//        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
//            motors = new Motors(op);
            servos = new Servos(op);

            telemetry.addLine("Teleskope inited");
        }
        public Motors motors;
        public Servos servos;

        public void setTelescope(double power, double powerInTake, double sPosUlitka, double sPosRamp, double sPosBaraban){
//            motors.setPower(power, powerInTake);

            servos.setBaraban(sPosBaraban);
            servos.setUlitka(sPosUlitka);
            servos.setRamp(sPosRamp);
        }
        public class Motors {
            public Motors(OpMode op){
//                selfData = new SelfData();
//
////                inTake = op.hardwareMap.get(DcMotor.class, "inTake");
////                flyWheelRight = op.hardwareMap.get(DcMotor.class, "flyWheelRight");
////                flyWheelLeft = op.hardwareMap.get(DcMotor.class, "flyWheelLeft");
//
//                inTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                flyWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                flyWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
////                inTake.setDirection(DcMotorSimple.Direction.FORWARD);
////                flyWheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//                inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                flyWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                flyWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.addLine("Motors on collector inited");
            }
            private  DcMotor inTake;
            private  DcMotor flyWheelRight;
            private  DcMotor flyWheelLeft;
            public  SelfData selfData;
            public void setPower(double power1, double powerInTake){
                inTake.setPower(powerInTake);

                flyWheelLeft.setPower(power1 * (-1));
                flyWheelRight.setPower(power1 * 1);
            }

            public class SelfData{
                private double curHeight = OFFSET_FROM_LAND;
                private double leftEncOld;
                private double rightEncOld;

                public double getCurHeight(){//Чтобы высота всегда была вовремя высчитана нужно потсоянно вызывать её подсчёт
                    calculateHeight();

                    return curHeight;
                }
                private void calculateHeight(){

                }
                private double ticksToCM(double ticks){
                    return ticks / TICK_PER_CM_BARABAN;
                }

                public void showHeight(){
                    telemetry.addData("Height", getCurHeight() + " " + "См");
                }
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
                ramp = op.hardwareMap.get(Servo.class, "s0");
                baraban = op.hardwareMap.get(Servo.class, "s1");
                ulitka = op.hardwareMap.get(Servo.class, "s2");

                setRamp(0);//Устанавливаем в начальное положение
                setBaraban(0);
                setUlitka(0);

                telemetry.addLine("Servos inited");
            }
            private final Servo ulitka;
            private final Servo ramp;
            private final Servo baraban;

            public Servo getBaraban() {
                return baraban;
            }

            public Servo getRamp() {
                return ramp;
            }

            public Servo getUlitka() {
                return ulitka;
            }

            private void setUlitka(double pos){
                ulitka.setPosition(pos);
            }
            private void setRamp(double pos){
                ramp.setPosition(pos);
            }

            private void setBaraban(double pos){
                baraban.setPosition(pos);
            }

            public void showServosPos(){
                telemetry.addLine("Servos statements")
                        .addData("\nzahvat", ulitka.getPosition())
                        .addData("\nhook", ramp.getPosition())
                        .addData("\nrotate", baraban.getPosition());
                telemetry.addLine();
            }
        }

    }
}

