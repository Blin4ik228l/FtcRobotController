package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.ExecutableModule;
import org.firstinspires.ftc.teamcode.Modules.MainModule;

public class Servos extends MainModule {
    public Servos(OpMode op){
        super(op.telemetry);

        pusher = op.hardwareMap.get(Servo.class, "pusher");
        baraban = op.hardwareMap.get(Servo.class, "baraban");
        angle = op.hardwareMap.get(Servo.class, "angle");
        runtime = new ElapsedTime();

        //Устанавливаем в начальное положение
        pusher.setPosition(PUSHER_START_POS);
        angle.setPosition(ANGLE_ENDING_POS);

        while (true) {
            if (!(runtime.seconds() < 1)) break;
        }

        baraban.setPosition(BARABAN_START_POS);

        runTimeBaraban = new ElapsedTime();
        runTimeAngle = new ElapsedTime();
        runTimePusher = new ElapsedTime();

        telemetry.addLine("Servos inited");
    }
    private final ElapsedTime runtime;
    private final Servo angle;
    private final Servo pusher;
    private final Servo baraban;
    public double curAnglePos, curPusherPos, curBarabanPos;
    public double anglePos = BARABAN_START_POS;
    public double pusherPos = PUSHER_START_POS;
    public double barabanPos = ANGLE_ENDING_POS;
    public Servo getBaraban() {
        return baraban;
    }
    public Servo getPusher() {
        return pusher;
    }
    public Servo getAngle() {
        return angle;
    }
    public BarabanState barabanState;
    public PusherState pusherState;
    public AngleState angleState;
    public ElapsedTime runTimeBaraban, runTimeAngle, runTimePusher;
    public enum BarabanState{
        inPos, notInPos
    }
    public enum PusherState{
        inPos, notInPos
    }
    public enum AngleState{
        inPos, notInPos
    }

    @Override
    public void update() {

    }

    @Override
    public void execute() {
        if(curAnglePos == anglePos && curBarabanPos == barabanPos && curPusherPos == pusherPos) return;

        angle.setPosition(anglePos);
        pusher.setPosition(pusherPos);
        baraban.setPosition(barabanPos);

        curAnglePos = angle.getPosition();
        curPusherPos = pusher.getPosition();
        curBarabanPos = baraban.getPosition();

        runTimeBaraban.reset();
        runTimePusher.reset();
        runTimeAngle.reset();

        barabanState = BarabanState.inPos;
        angleState = AngleState.inPos;
        pusherState = PusherState.inPos;
    }

    @Override
    public void showData(){
        telemetry.addLine("===SERVOS===");
        telemetry.addData("Pos","A:%s P:%s B:%s",curAnglePos, curPusherPos, curBarabanPos);
        telemetry.addData("Baraban time", runTimeBaraban);
        telemetry.addLine();
    }
}