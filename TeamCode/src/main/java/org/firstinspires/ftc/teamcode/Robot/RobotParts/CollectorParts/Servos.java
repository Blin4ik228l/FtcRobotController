package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.MainModule;
import org.firstinspires.ftc.teamcode.Modules.Module;

public class Servos extends Module {
    public Servos(OpMode op){
        super(op.telemetry);

        pusher = op.hardwareMap.get(Servo.class, "pusher");
        baraban = op.hardwareMap.get(Servo.class, "baraban");
        baraban2 = op.hardwareMap.get(Servo.class, "baraban2");
        angle = op.hardwareMap.get(Servo.class, "angle");
        runtime = new ElapsedTime();

        //Устанавливаем в начальное положение
        pusher.setPosition(PUSHER_START_POS);
        angle.setPosition(ANGLE_START_POS);

        while (true) {
            if (!(runtime.seconds() < 1)) break;
        }
        baraban.setPosition(BARABAN2_START_POS);
        baraban.setPosition(BARABAN_START_POS);

        runTimeBaraban = new ElapsedTime();
        runTimeAngle = new ElapsedTime();
        runTimePusher = new ElapsedTime();

        telemetry.addLine("Servos inited");
    }
    private final ElapsedTime runtime;
    private final Servo angle;
    private final Servo pusher;
    private final Servo baraban, baraban2;
    public double curAnglePos, curPusherPos, curBarabanPos;
    public double targetAnglePos = BARABAN_START_POS;
    public double targetPusherPos = PUSHER_START_POS;
    public double targetBarabanPos = ANGLE_ENDING_POS;
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

    public void setBaraban(double targetBarabanPos){
        if(curBarabanPos == targetBarabanPos) return;

        baraban.setPosition(targetBarabanPos);
        baraban2.setPosition(1 - targetBarabanPos);

        curBarabanPos = baraban.getPosition();

        runTimeBaraban.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public void setPusher(double targetPusherPos){
        if(curPusherPos == targetPusherPos) return;
        pusher.setPosition(targetPusherPos);

        curPusherPos = pusher.getPosition();

        runTimePusher.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    public void setAngle(double targetAnglePos){
        if(curAnglePos == targetAnglePos) return;

        angle.setPosition(targetAnglePos);

        curAnglePos = angle.getPosition();

        runTimeAngle.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    @Override
    public void showData(){
        telemetry.addLine("===SERVOS===");
        telemetry.addData("Pos","A:%s P:%s B:%s",curAnglePos, curPusherPos, curBarabanPos);
        telemetry.addData("Baraban time", runTimeBaraban);
        telemetry.addData("Pusher time", runTimePusher);
        telemetry.addLine();
    }
}