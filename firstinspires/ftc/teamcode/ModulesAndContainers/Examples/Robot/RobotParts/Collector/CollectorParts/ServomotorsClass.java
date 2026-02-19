package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public class ServomotorsClass extends Module {
    private Servo angle;
    private Servo pusherHor, pusherVer;
    private Servo baraban, baraban2;
    public ElapsedTime runTimeBaraban, runTimeAngle, runTimePusherHor, runTimePusherVer;
    public ServomotorsClass(OpMode op){
        super(op);
        try {

        } catch (Exception e) {
            isInizialized = false;
            return;
        }
        pusherHor = hardwareMap.get(Servo.class, "pusher");
        pusherVer = hardwareMap.get(Servo.class, "pusher2");
        baraban = hardwareMap.get(Servo.class, "baraban");
        baraban2 = hardwareMap.get(Servo.class, "baraban2");
        angle = hardwareMap.get(Servo.class, "angle");

        runTimeBaraban = new ElapsedTime();
        runTimeAngle = new ElapsedTime();
        runTimePusherHor = new ElapsedTime();
        runTimePusherVer = new ElapsedTime();

        //Устанавливаем в начальное положение
        setPusherVer(PUSHERVER_START_POS);
        setPusherHor(PUSHER_START_POS);
        setAngle(ANGLE_LOWER_POS);
        setBaraban(BARABAN_CELL0_POS);

        telemetry.addLine("Servos Inited");
    }
    public void init(){
        pusherHor = hardwareMap.get(Servo.class, "pusher");
        pusherVer = hardwareMap.get(Servo.class, "pusher2");
        baraban = hardwareMap.get(Servo.class, "baraban");
        baraban2 = hardwareMap.get(Servo.class, "baraban2");
        angle = hardwareMap.get(Servo.class, "angle");

        runTimeBaraban = new ElapsedTime();
        runTimeAngle = new ElapsedTime();
        runTimePusherHor = new ElapsedTime();
        runTimePusherVer = new ElapsedTime();

        //Устанавливаем в начальное положение
        setPusherVer(PUSHERVER_START_POS);
        setPusherHor(PUSHER_START_POS);
        setAngle(ANGLE_LOWER_POS);
        setBaraban(BARABAN_CELL0_POS);
    }
    public Servo getBaraban() {
        return baraban;
    }
    public Servo getPusherHor() {
        return pusherHor;
    }
    public Servo getAngle() {
        return angle;
    }
    public double targetAngle, curAngle;
    public double barabanDelay, pusherHorDelay, pusherVerDelay;
    public double curAnglePos = -1, curPusherHorPos = -1, curBarabanPos = -1, curPusherVerPos = -1;
    public void calculateDelayBaraban(double targPos){
        int curCell;
        int targCell;

        if(curBarabanPos == BARABAN_CELL0_POS) curCell = 0;
        else if (curBarabanPos == BARABAN_CELL1_POS) curCell = 1;
        else curCell = 2;

        if(targPos == BARABAN_CELL0_POS) targCell = 0;
        else if (targPos == BARABAN_CELL1_POS) targCell = 1;
        else targCell = 2;

        int way = Math.abs(targCell - curCell);

        if (runTimeBaraban.seconds() > barabanDelay){
            switch (way){
                case 0:
                    barabanDelay = 0;
                    break;
                case 1:
                    barabanDelay = BARABAN_DELAY_NEAR;
                    break;
                case 2:
                    barabanDelay = BARABAN_DELAY_FAR;
            }
        }
    }

    public void calculateDelayPusher(double targPos){
        int curPosNum;
        int targPosNum;

        if(curPusherHorPos == PUSHER_START_POS) curPosNum = 0;
        else if (curPusherHorPos == PUSHER_PREFIRE_POS) curPosNum = 1;
        else curPosNum = 2;

        if(targPos == PUSHER_START_POS) targPosNum = 0;
        else if (targPos == PUSHER_PREFIRE_POS) targPosNum = 1;
        else targPosNum = 2;

        int way = Math.abs(targPosNum - curPosNum);

        if (runTimePusherHor.seconds() > pusherHorDelay){
            switch (way){
                case 0:
                    pusherHorDelay = 0;
                    break;
                case 1:
                    pusherHorDelay = PUSHERHOR_DELAY_NEAR;
                    break;
                case 2:
                    pusherHorDelay = PUSHERHOR_DELAY_FAR;
            }
        }
    }
    public void setBaraban(double targetBarabanPos){
        if(curBarabanPos == targetBarabanPos) return;
        baraban.setPosition(targetBarabanPos);
        baraban2.setPosition(targetBarabanPos);
        calculateDelayBaraban(targetBarabanPos);

        curBarabanPos = baraban.getPosition();

        curBarabanPos = Math.round(curBarabanPos * Math.pow(10, 2)) / Math.pow(10, 2);
        runTimeBaraban.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public void setPusherHor(double targetPusherPos){
        if(curPusherHorPos == targetPusherPos) return;
//        setPusherVer(targetPusherPos);
//
//        runTimePusherHor.reset();//Обнуляем время с момента попадания программы в эту часть
//        if(runTimePusherVer.seconds() < pusherVerDelay) return;

        pusherHor.setPosition(targetPusherPos);
        calculateDelayPusher(targetPusherPos);

        curPusherHorPos = pusherHor.getPosition();

        runTimePusherHor.reset();//Обнуляем время с момента попадания программы в эту часть
    }
    public void setPusherVer(double targetPusherPos){
//        if (targetPusherPos == PUSHERHOR_ENDING_POS){
//            targetPusherPos = PUSHERVER_ENDING_POS;
//            pusherVerDelay = PUSHERVER_DELAY;
//        }else {
//            pusherVerDelay = 0.0;
//            targetPusherPos = PUSHERVER_START_POS;}

        if(targetPusherPos == curPusherVerPos) return;
        pusherVer.setPosition(targetPusherPos);

        curPusherVerPos = pusherVer.getPosition();
        runTimePusherVer.reset();
    }

    public void setAngle(double targetAnglePos){
        targetAnglePos = Range.clip(targetAnglePos, ANGLE_UPPER_POS, ANGLE_LOWER_POS);

        if (targetAnglePos == curAnglePos) return;
        angle.setPosition(targetAnglePos);
        curAnglePos = angle.getPosition();

        curAngle = fromPosToAngle(curAnglePos);

        runTimeAngle.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public double fromPosToAngle(double curPos){
        double angle = MAX_ANGLE - curPos / (185 / 23) * 270;

        return Math.round(angle * Math.pow(10, 2)) / Math.pow(10, 2);
    }

    @Override
    public void showData(){
        telemetry.addLine("===SERVOMOTORS===");
        telemetry.addLine("Baraban");
        telemetry.addData("BPos", curBarabanPos);
        telemetry.addData("Const","C0:%s C1:%s C2:%s", BARABAN_CELL0_POS, BARABAN_CELL1_POS, BARABAN_CELL2_POS);
        telemetry.addLine("Angle");
        telemetry.addData("APos", curAnglePos);
        telemetry.addData("Current angle", curAngle);
        telemetry.addLine("Pushers");
        telemetry.addData("Poses","Hor:%s Ver:%s", curPusherHorPos, curPusherVerPos);

        telemetry.addData("Delays","B:%s P:%s", barabanDelay, pusherHorDelay);
//        telemetry.addData("Angle time", runTimeAngle);
//        telemetry.addData("Baraban time", runTimeBaraban);
//        telemetry.addData("Pusher time", runTimePusherHor);
        telemetry.addLine();
    }
}