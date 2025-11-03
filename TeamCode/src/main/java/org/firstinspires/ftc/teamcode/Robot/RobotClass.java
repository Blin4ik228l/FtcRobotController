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

        driveTrain = new MecanumDrivetrain(op, this);
//        teleSkope = new TeleSkope(op);// Пока не на роботе
    }
    public MecanumDrivetrain driveTrain;
    public Telescope telescope;
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

    public static class Telescope extends Module{
        public Telescope(OpMode op) {
            super(op.telemetry);
            lift = new Lift();
            servos = new Servos(op);

            telemetry.addLine("Teleskope inited");
        }
        public Lift lift;
        public Servos servos;

        public void setTelescope(double power, boolean isToPos, double height, double sPosHorizontal, double sPosHook, double sPosFlip){
            lift.setPower(power, isToPos, height);

            servos.setFlip(sPosFlip);
//            servos.setHorizontal(sPosHorizontal);
            servos.setHook(sPosHook);
        }
        public class Lift{
//            public Lift(OpMode op){
//                selfData = new SelfData();
//                left = op.hardwareMap.get(DcMotor.class, "leftTele");
//                right = op.hardwareMap.get(DcMotor.class, "rightTele");
//
//                left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                left.setDirection(DcMotorSimple.Direction.FORWARD);
//                right.setDirection(DcMotorSimple.Direction.REVERSE);
//
//                left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                telemetry.addLine("Lift inited");
//            }
            private  DcMotor left;
            private  DcMotor right;
            public  SelfData selfData;
            public void setPower(double power, boolean isToPos, double height){
//            if(isToPos){//Если у нас телескоп в режиме "доезда" до точки (этот режим нужен по идее только для автономки)
//                if(selfData.curHeight != height){
//                    double dir = Math.signum(selfData.getCurHeight() - height);
//
//                    if(power == 0){//Данная строчка нужно чтобы поддерживать телескоп на определённом уровне, даже когда по аргумантам не задано
//                        power = 0.05;
//                    }
//
//                    left.setPower(power * dir);
//                    right.setPower(-power * dir);
//                }
//            }
//
//            if(!isToPos && height > OFFSET_FROM_LAND) {//Это режим телескопа позволяет поднимать его по уровням, как-бы создавая границы (практиковался в упраляемом режиме)
//                if(power >= 0 && selfData.curHeight != height || power <= 0 && selfData.curHeight != OFFSET_FROM_LAND){
//                    left.setPower(power);
//                    right.setPower(-power);
//                }
//            }
//
//            if(!isToPos && height == 0){//Обычная подача напряжение на моторы телескопа
//                left.setPower(power);
//                right.setPower(-power);
//            }
                left.setPower(power);
                right.setPower(power);
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
                    double leftEnc = ticksToCM(left.getCurrentPosition());
                    double deltaLeftEnc = leftEnc - leftEncOld;
                    leftEncOld = leftEnc;

                    double rightEnc = ticksToCM(right.getCurrentPosition());
                    double deltaRightEnc = rightEnc - rightEncOld;
                    rightEncOld = rightEnc;

                    curHeight += (deltaRightEnc + deltaLeftEnc)/2.0;
                }
                private double ticksToCM(double ticks){
                    return ticks / TICK_PER_CM_BARABAN;
                }

                public void showHeight(){
                    telemetry.addData("Height", getCurHeight() + " " + "См");
                }
            }
        }
        public class Servos{
            public Servos(OpMode op){
                hook = op.hardwareMap.get(Servo.class, "s0");
                flip = op.hardwareMap.get(Servo.class, "s1");


//                setHook(OPEN_POS_HOOK);//Устанавливаем в начальное положение
//                setHorizontal(OPEN_POS_HORIZONTAL);
//                setFlip(MIDLE_POS_FLIP);

                telemetry.addLine("Servos inited");
            }
            private  Servo horizontal;
            private final Servo hook;
            private final Servo flip;

            public Servo getFlip() {
                return flip;
            }

            public Servo getHook() {
                return hook;
            }

            public Servo getHorizontal() {
                return horizontal;
            }

            private void setHorizontal(double pos){
                horizontal.setPosition(pos);
            }
            private void setHook(double pos){
                hook.setPosition(pos);
            }

            private void setFlip(double pos){
                flip.setPosition(pos);
            }

            public void showServosPos(){
                telemetry.addLine("Servos statements")
                        .addData("\nzahvat", horizontal.getPosition())
                        .addData("\nhook", hook.getPosition())
                        .addData("\nrotate", flip.getPosition());
                telemetry.addLine();
            }
        }

    }
}

