package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class ServomotorsClass extends Module {
    public ServomotorsClass(OpMode op){
        super(op.telemetry);

        pusherHor = op.hardwareMap.get(Servo.class, "pusher");
        pusherVer = op.hardwareMap.get(Servo.class, "pusher2");
        baraban = op.hardwareMap.get(Servo.class, "baraban");
        baraban2 = op.hardwareMap.get(Servo.class, "baraban2");
        angle = op.hardwareMap.get(Servo.class, "angle");

        runTimeBaraban = new ElapsedTime();
        runTimeAngle = new ElapsedTime();
        runTimePusherHor = new ElapsedTime();
        runTimePusherVer = new ElapsedTime();

        //Устанавливаем в начальное положение
        setPusherVer(PUSHERVER_START_POS);
        setPusherHor(PUSHER_START_POS);
        setAngle(ANGLE_ENDING_POS);
        setBaraban(BARABAN_CELL0_POS);

        telemetry.addLine("Servos Inited");
    }
    private final Servo angle;
    private final Servo pusherHor, pusherVer;
    private final Servo baraban, baraban2;
    public double curAnglePos = -1, curPusherHorPos = -1, curBarabanPos = -1, curPusherVerPos = -1;
    public Servo getBaraban() {
        return baraban;
    }
    public Servo getPusherHor() {
        return pusherHor;
    }
    public Servo getAngle() {
        return angle;
    }
    public ElapsedTime runTimeBaraban, runTimeAngle, runTimePusherHor, runTimePusherVer;
    public double targetAngle;
    public double barabanDelay = 0;
    public double pusherDelay;
    public double pusherVerDelay = 0;
    public enum AngleStates{
        Ready,
        Unready
    }
    public AngleStates angleStates = AngleStates.Unready;
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

        if (runTimePusherHor.seconds() > pusherDelay){
            switch (way){
                case 0:
                    pusherDelay = 0;
                    break;
                case 1:
                    pusherDelay = PUSHERHOR_DELAY_NEAR;
                    break;
                case 2:
                    pusherDelay = PUSHERHOR_DELAY_FAR;
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
        setPusherVer(targetPusherPos);

        runTimePusherHor.reset();//Обнуляем время с момента попадания программы в эту часть
        if(runTimePusherVer.seconds() < pusherVerDelay) return;

        pusherHor.setPosition(targetPusherPos);
        calculateDelayPusher(targetPusherPos);

        curPusherHorPos = pusherHor.getPosition();

        runTimePusherHor.reset();//Обнуляем время с момента попадания программы в эту часть
    }
    public void setPusherVer(double targetPusherPos){
        if(targetPusherPos == PUSHERHOR_ENDING_POS){
            targetPusherPos = PUSHERVER_ENDING_POS;
            pusherVerDelay = PUSHERVER_DELAY;
        }else {
            pusherVerDelay = 0.0;
            targetPusherPos = PUSHERVER_START_POS;}

        if(targetPusherPos == curPusherVerPos) return;
        pusherVer.setPosition(targetPusherPos);

        curPusherVerPos = pusherVer.getPosition();
        runTimePusherVer.reset();
    }

    public void setAngle(double targetAnglePos){
        targetAngle = fromPosToAngle(targetAnglePos);

        angle.setPosition(targetAnglePos);
        curAnglePos = angle.getPosition();

        double errorAngle = targetAngle - fromPosToAngle(curAnglePos);
        angleStates = Math.abs(errorAngle) < 0.9 ? ServomotorsClass.AngleStates.Ready : ServomotorsClass.AngleStates.Unready;

        if(angleStates == AngleStates.Unready) runTimeAngle.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public double fromPosToAngle(double curPos){
        double angle = MAX_ANGLE - curPos / (185 / 23) * 270;

        return Math.round(angle * Math.pow(10, 2)) / Math.pow(10, 2);
    }

    @Override
    public void showData(){
        telemetry.addLine("===SERVOS===");
        telemetry.addData("Delays","B:%s P:%s", barabanDelay, pusherDelay);
        telemetry.addData("Pos","A:%s P:%s B:%s",curAnglePos, curPusherHorPos, curBarabanPos);
        telemetry.addData("Const","%s %s %s", BARABAN_CELL0_POS, BARABAN_CELL1_POS, BARABAN_CELL2_POS);
        telemetry.addData("Current angle", "%s", fromPosToAngle(curAnglePos));
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Angle time", runTimeAngle);
        telemetry.addData("Baraban time", runTimeBaraban);
        telemetry.addData("Pusher time", runTimePusherHor);
        telemetry.addLine();
    }
}