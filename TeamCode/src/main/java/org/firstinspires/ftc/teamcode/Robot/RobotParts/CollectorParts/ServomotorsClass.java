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
        setAngle(ANGLE_START_POS);
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
                    barabanDelay = 0.15;
                    break;
                case 2:
                    barabanDelay = 0.25;
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
                    pusherDelay = 0.1;
                    break;
                case 2:
                    pusherDelay = 0.2;
            }
        }
    }
    public void setBaraban(double targetBarabanPos){
        calculateDelayBaraban(targetBarabanPos);

        if(curBarabanPos == targetBarabanPos) return;
        baraban.setPosition(targetBarabanPos);
        baraban2.setPosition(targetBarabanPos);

        curBarabanPos = baraban.getPosition();

        runTimeBaraban.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public void setPusherHor(double targetPusherPos){
        calculateDelayPusher(targetPusherPos);

        if(curPusherHorPos == targetPusherPos) return;
        setPusherVer(targetPusherPos);

        pusherHor.setPosition(targetPusherPos);

        curPusherHorPos = pusherHor.getPosition();

        runTimePusherHor.reset();//Обнуляем время с момента попадания программы в эту часть
    }
    public void setPusherVer(double targetPusherPos){
        if(targetPusherPos == PUSHERHOR_ENDING_POS){

            targetPusherPos = PUSHERVER_ENDING_POS;
        }else targetPusherPos = PUSHERVER_START_POS;

        if(targetPusherPos == curPusherVerPos) return;
        pusherVer.setPosition(targetPusherPos);

        curPusherVerPos = pusherVer.getPosition();
        runTimePusherVer.reset();
    }

    public void setAngle(double targetAnglePos){
        angleStates = AngleStates.Ready;
        targetAngle = targetAnglePos;
        if(Math.abs(fromPosToAngle(curAnglePos) - fromPosToAngle(targetAnglePos)) < 0.9) return;
        angle.setPosition(targetAnglePos);
        curAnglePos = angle.getPosition();

        angleStates = AngleStates.Unready;

        runTimeAngle.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public double fromPosToAngle(double curPos){
        return MAX_ANGLE - curPos / (185 / 23) * 270;
    }

    @Override
    public void showData(){
        telemetry.addLine("===SERVOS===");
        telemetry.addData("Delays","B:%s P:%s", barabanDelay, pusherDelay);
        telemetry.addData("Pos","A:%s P:%s B:%s",curAnglePos, curPusherHorPos, curBarabanPos);
        telemetry.addData("Current angle", "%s", fromPosToAngle(curAnglePos));
        telemetry.addData("Target Angle", fromPosToAngle(targetAngle));
        telemetry.addData("Angle time", runTimeAngle);
        telemetry.addData("Baraban time", runTimeBaraban);
        telemetry.addData("Pusher time", runTimePusherHor);
        telemetry.addLine();
    }
}