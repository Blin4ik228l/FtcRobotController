package org.firstinspires.ftc.teamcode.Modules.Handlers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.TeleSkope;

public class TeleSkopeHandler extends Handler {
    public TeleSkopeHandler(TeleSkope teleSkope, Telemetry telemetry) {
        super(telemetry);
        this.teleSkope = teleSkope;
    }
    public void setArgs(Args.LiftArgs liftArgs){
        this.liftArgs = liftArgs;
    }
    public void setArgs(Args.ServoArgs servoArgs){
        this.servoArgs = servoArgs;
    }
    public TeleSkope teleSkope;
    public Args.LiftArgs liftArgs = null;
    public Args.ServoArgs servoArgs = null;

    @Override
    public void execute() {
        double power = 0;
        double targetHeight = 0;
        double horizontalPos = 0;
        double hookPos = 0;
        double flipPos = 0;

        if(liftArgs != null){
            power = liftArgs.power;
            targetHeight = liftArgs.height;
        }

        if(servoArgs != null){
            if(servoArgs.servoName.equals("hook")){
                hookPos = servoArgs.servoPos;
                horizontalPos = teleSkope.servos.getHorizontal().getPosition();
                flipPos = teleSkope.servos.getFlip().getPosition();
            }
            if(servoArgs.servoName.equals("flip")){
                flipPos = servoArgs.servoPos;
                horizontalPos = teleSkope.servos.getHorizontal().getPosition();
                hookPos = teleSkope.servos.getHook().getPosition();
            }
            if(servoArgs.servoName.equals("horizontal")){
                horizontalPos = servoArgs.servoPos;
                hookPos = teleSkope.servos.getHook().getPosition();
                flipPos = teleSkope.servos.getFlip().getPosition();
            }
        }else{
            horizontalPos = teleSkope.servos.getHorizontal().getPosition();
            hookPos = teleSkope.servos.getHook().getPosition();
            flipPos = teleSkope.servos.getFlip().getPosition();
        }

        teleSkope.setTeleskope(power, true, targetHeight, horizontalPos, hookPos, flipPos);

        if(teleSkope.lift.selfData.getCurHeight() == liftArgs.height
                && teleSkope.servos.getHorizontal().getPosition() == horizontalPos
                && teleSkope.servos.getHook().getPosition() == hookPos
                && teleSkope.servos.getFlip().getPosition() == flipPos  ||
                power == 0 && targetHeight == 0 && teleSkope.servos.getHorizontal().getPosition() == horizontalPos
                && teleSkope.servos.getHook().getPosition() == hookPos
                && teleSkope.servos.getFlip().getPosition() == flipPos ){
            isDone = true;
        }

        showData();
    }

    @Override
    public void showData() {
        teleSkope.lift.selfData.showHeight();
        teleSkope.servos.showServosPos();
    }
}
