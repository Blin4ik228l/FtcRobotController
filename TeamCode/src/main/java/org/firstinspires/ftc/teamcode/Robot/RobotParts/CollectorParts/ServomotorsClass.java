package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class ServomotorsClass extends Module {
    public ServomotorsClass(OpMode op){
        super(op.telemetry);

        pusher = op.hardwareMap.get(Servo.class, "pusher");
        baraban = op.hardwareMap.get(Servo.class, "baraban");
        baraban2 = op.hardwareMap.get(Servo.class, "baraban2");
        angle = op.hardwareMap.get(Servo.class, "angle");

        runTimeBaraban = new ElapsedTime();
        runTimeAngle = new ElapsedTime();
        runTimePusher = new ElapsedTime();

        //Устанавливаем в начальное положение
        setPusher(PUSHER_START_POS);
        setAngle(ANGLE_START_POS);
        setBaraban(BARABAN_CELL0_POS);

        telemetry.addLine("Servos Inited");
    }
    private final Servo angle;
    private final Servo pusher;
    private final Servo baraban, baraban2;
    public double curAnglePos = -1, curPusherPos = -1, curBarabanPos = -1;
    public Servo getBaraban() {
        return baraban;
    }
    public Servo getPusher() {
        return pusher;
    }
    public Servo getAngle() {
        return angle;
    }
    public ElapsedTime runTimeBaraban, runTimeAngle, runTimePusher;
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

    public void calculateDelayPusher(double targPos){
        int curPosNum;
        int targPosNum;

        if(curPusherPos == PUSHER_START_POS) curPosNum = 0;
        else if (curPusherPos == PUSHER_PREFIRE_POS) curPosNum = 1;
        else curPosNum = 2;

        if(targPos == PUSHER_START_POS) targPosNum = 0;
        else if (targPos == PUSHER_PREFIRE_POS) targPosNum = 1;
        else targPosNum = 2;

        int way = Math.abs(targPosNum - curPosNum);

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
    public void setBaraban(double targetBarabanPos){
        if(curBarabanPos == targetBarabanPos) return;
        baraban.setPosition(targetBarabanPos);
        baraban2.setPosition(1 - targetBarabanPos);

        curBarabanPos = baraban.getPosition();

        calculateDelayBaraban(targetBarabanPos);
        runTimeBaraban.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public void setPusher(double targetPusherPos){
        if(curPusherPos == targetPusherPos) return;
        pusher.setPosition(targetPusherPos);

        curPusherPos = pusher.getPosition();

        calculateDelayPusher(targetPusherPos);
        runTimePusher.reset();//Обнуляем время с момента попадания программы в эту часть
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
        return  -curPos / (185 / 23) * 270 + 43;
    }

    @Override
    public void showData(){
        telemetry.addLine("===SERVOS===");
        telemetry.addData("Pos","A:%s P:%s B:%s",curAnglePos, curPusherPos, curBarabanPos);
        telemetry.addData("Current angle", "%s", fromPosToAngle(curAnglePos));
        telemetry.addData("Target Angle", fromPosToAngle(targetAngle));
        telemetry.addData("Angle time", runTimeAngle);
        telemetry.addData("Baraban time", runTimeBaraban);
        telemetry.addData("Pusher time", runTimePusher);
        telemetry.addLine();
    }
}