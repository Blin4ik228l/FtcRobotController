package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Module;

import java.util.HashMap;

public class Servos extends Module {
    public Servos(OpMode op){
        super(op.telemetry);

        pusher = op.hardwareMap.get(Servo.class, "pusher");
        baraban = op.hardwareMap.get(Servo.class, "baraban");
        angle = op.hardwareMap.get(Servo.class, "angle");
        runtime = new ElapsedTime();

        //Устанавливаем в начальное положение
        setPusher(PUSHER_START_POS);
        setAngle(ANGLE_START_POS);

        while (runtime.seconds() < 1) {}
        setBaraban(BARABAN_START_POS);

        telemetry.addLine("Servos inited");
    }

    ElapsedTime runtime;
    private final Servo angle;
    private final Servo pusher;
    private final Servo baraban;

    public int loadedArtifactColor = 0;

    public boolean isRotating = false;
    public Servo getBaraban() {
        return baraban;
    }

    public Servo getPusher() {
        return pusher;
    }

    public Servo getAngle() {
        return angle;
    }

    private void setAngle(double pos){
        angle.setPosition(pos);
    }
    private void setPusher(double pos){
        pusher.setPosition(pos);
    }
    private void setBaraban(double pos){
        baraban.setPosition(pos);
    }

    public void showServosPos(){
        telemetry.addLine("Servos statements")
                .addData("\nangle", angle.getPosition())
                .addData("\npusher", pusher.getPosition())
                .addData("\nbaraban", baraban.getPosition());
        telemetry.addLine();
    }


}