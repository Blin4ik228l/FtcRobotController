package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;

public class ServosService implements Module, CONSTSTELESKOPE {
    private final OpMode op;

    private Servo hook;
    private Servo horizontal;
    private Servo flip;

    public ServosService(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        horizontal = op.hardwareMap.get(Servo.class, "horizontal");
        hook = op.hardwareMap.get(Servo.class, "hook");
        flip = op.hardwareMap.get(Servo.class, "flip");

        setHookStartPos();
//        setHorizontalStartPos();
//        setFlipStartPos();

        op.telemetry.addLine("Servos Inited");
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
        op.telemetry.addData("",hook.getController().getPwmStatus());
        hook.getController().pwmEnable();
        hook.getController().setServoPosition(5,CLOSE_POS_HOOK);
    }

    public void setFlipStartPos(){
        flip.setPosition(TAKE_POS_FLIP);
    }

    public synchronized void getServosPos(){
        op.telemetry.addLine("Servos")
                .addData("\nFlip", flip.getPosition())
                .addData("\nHook", hook.getController().getServoPosition(5))
                .addData("\nHorizontal", horizontal.getPosition());
        op.telemetry.addLine();
    }
}
