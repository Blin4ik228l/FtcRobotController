package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Module;

public class TeleSkope extends Module {
    public TeleSkope(OpMode op) {
        super(op.telemetry);
        lift = new Lift(op);
        servos = new Servos(op);

        telemetry.addLine("Teleskope inited");
    }
    public Lift lift;
    public Servos servos;

    public void setTeleskope(double power, boolean isToPos, double height, double sPosHorizontal, double sPosHook, double sPosFlip){
        lift.setPower(power, isToPos, height);

        servos.setFlip(sPosFlip);
        servos.setHorizontal(sPosHorizontal);
        servos.setHook(sPosHook);
    }
    public class Lift{
       public Lift(OpMode op){
           selfData = new SelfData();
           left = op.hardwareMap.get(DcMotor.class, "left");
           right = op.hardwareMap.get(DcMotor.class, "right");

           left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

           left.setDirection(DcMotorSimple.Direction.FORWARD);
           right.setDirection(DcMotorSimple.Direction.REVERSE);

           left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

           telemetry.addLine("Lift inited");
        }
        private final DcMotor left;
        private final DcMotor right;
        public final SelfData selfData;

        public void setPower(double power, boolean isToPos, double height){
            if(isToPos){//Если у нас телескоп в режиме "доезда" до точки (этот режим нужен по идее только для автономки)
                if(selfData.curHeight != height){
                    double dir = Math.signum(selfData.getCurHeight() - height);

                    if(power == 0){//Данная строчка нужно чтобы поддерживать телескоп на определённом уровне, даже когда по аргумантам не задано
                        power = 0.05;
                    }

                    left.setPower(power * dir);
                    right.setPower(-power * dir);
                }
            }

            if(!isToPos && height > OFFSET_FROM_LAND) {//Это режим телескопа позволяет поднимать его по уровням, как-бы создавая границы (практиковался в упраляемом режиме)
                if(power >= 0 && selfData.curHeight != height || power <= 0 && selfData.curHeight != OFFSET_FROM_LAND){
                    left.setPower(power);
                    right.setPower(-power);
                }
            }

            if(!isToPos && height == 0){//Обычная подача напряжение на моторы телескопа
                left.setPower(power);
                right.setPower(-power);
            }
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
            hook = op.hardwareMap.get(Servo.class, "hook");
            flip = op.hardwareMap.get(Servo.class, "rotate");
            horizontal = op.hardwareMap.get(Servo.class, "zahvat");

            setHook(1);//Устанавливаем в начальное положение
            setHorizontal(1);
            setFlip(1);

            telemetry.addLine("Servos inited");
        }
        private final Servo horizontal;
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
