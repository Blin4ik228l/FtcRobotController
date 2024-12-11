package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;

public class ServosService implements Module, CONSTSTELESKOPE {
    private final OpMode op;

    private Servo hook;
    private Servo horizontal;

    public ServosService(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        horizontal = op.hardwareMap.get(Servo.class, "horizontal");
        hook = op.hardwareMap.get(Servo.class, "hook");

        setHookStartPos();
        setHorizontalStartPos();
    }

    public Servo getHook() {
        return hook;
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

    public void setServosStartPos(){}
}
