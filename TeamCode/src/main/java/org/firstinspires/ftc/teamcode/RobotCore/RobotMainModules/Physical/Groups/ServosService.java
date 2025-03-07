package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

public class ServosService implements Module, ConstsTeleskope {
    private final OpMode op;

    private Servo hook;
    private Servo horizontal;
    private Servo flip;
    private Servo left;
    private Servo right;

    public enum servoPos{
        UP,
        DOWN
    }
    public ServosService(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        horizontal = op.hardwareMap.get(Servo.class, "horizontal");
        hook = op.hardwareMap.get(Servo.class, "hook");
        flip = op.hardwareMap.get(Servo.class, "flip");

        left = op.hardwareMap.get(Servo.class, "left");
        right = op.hardwareMap.get(Servo.class, "right");

        setHookStartPos();
        setHorizontalStartPos();
        setFlipStartPos();

        setLeftStartPos();
        setRightStartPos();

        op.telemetry.addLine("Servos Inited");
    }


    public void setLeftStartPos(){
        left.setPosition(0);
    }
    public void setRightStartPos(){
        right.setPosition(1);
    }

    public Servo getLeft() {
        return left;
    }

    public Servo getRight() {
        return right;
    }

    public Servo getHook() {
        return hook;
    }

    public Servo getFlip() {
        return flip;
    }

    public Servo getHorizontal() {
        return horizontal;
    }

    public void setHorizontalStartPos(){
        horizontal.setPosition(CLOSE_POS_HORIZONTAL);
    }

    public void setHookStartPos() {
        hook.setPosition(CLOSE_POS_HOOK);
    }

    public void setFlipStartPos(){
        flip.setPosition(HANG_POS_FLIP);
    }

    public synchronized void getServosPos(){
        op.telemetry.addLine("Servos")
                .addData("\nFlip", flip.getPosition())
                .addData("\nHook", hook.getPosition())
                .addData("\nHorizontal", horizontal.getPosition());
        op.telemetry.addLine();
    }
}
