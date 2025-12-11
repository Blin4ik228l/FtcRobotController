package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class Servos extends Module {
    public Servos(OpMode op){
        super(op.telemetry);

        pusher = op.hardwareMap.get(Servo.class, "pusher");
        baraban = op.hardwareMap.get(Servo.class, "baraban");
        baraban2 = op.hardwareMap.get(Servo.class, "baraban2");
        angle = op.hardwareMap.get(Servo.class, "angle");
        runtime = new ElapsedTime();

        //Устанавливаем в начальное положение
        runTimeBaraban = new ElapsedTime();
        runTimeAngle = new ElapsedTime();
        runTimePusher = new ElapsedTime();

        setPusher(PUSHER_PREFIRE_POS);
        setAngle(ANGLE_START_POS);
        setBaraban(BARABAN_CELL0_POS);

        telemetry.addLine("Servos inited");
    }
    private final ElapsedTime runtime;
    private final Servo angle;
    private final Servo pusher;
    private final Servo baraban, baraban2;
    public double curAnglePos = -1, curPusherPos = -1, curBarabanPos = -1;
    public double targetAnglePos = BARABAN_CELL0_POS;
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

    public double targ;
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
        targ = targetAnglePos;
        if(curAnglePos == targetAnglePos) return;

        angle.setPosition(targetAnglePos);

        curAnglePos = angle.getPosition();

        runTimeAngle.reset();//Обнуляем время с момента попадания программы в эту часть
    }

    @Override
    public void showData(){
        telemetry.addLine("===SERVOS===");
        telemetry.addData("Pos","A:%s P:%s B:%s",curAnglePos, curPusherPos, curBarabanPos);
        telemetry.addData("Target Fire angle", targ);
        telemetry.addData("Angle time", runTimeAngle);
        telemetry.addData("Baraban time", runTimeBaraban);
        telemetry.addData("Pusher time", runTimePusher);
        telemetry.addLine();

//         * 270 / (185/23) + 75
    }
}