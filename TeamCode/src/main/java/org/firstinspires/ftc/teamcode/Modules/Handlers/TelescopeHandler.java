package org.firstinspires.ftc.teamcode.Modules.Handlers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

public class TelescopeHandler extends Handler {
    public TelescopeHandler(RobotClass.Telescope telescope, Telemetry telemetry) {
        super(telemetry);
        this.telescope = telescope;
    }
    public void setArgs(Args.LiftArgs liftArgs){
        this.liftArgs = liftArgs;
    }
    public void setArgs(Args.ServoArgs servoArgs){
        this.servoArgs = servoArgs;
    }
    public RobotClass.Telescope telescope;
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
                horizontalPos = telescope.servos.getHorizontal().getPosition();
                flipPos = telescope.servos.getFlip().getPosition();
            }
            if(servoArgs.servoName.equals("flip")){
                flipPos = servoArgs.servoPos;
                horizontalPos = telescope.servos.getHorizontal().getPosition();
                hookPos = telescope.servos.getHook().getPosition();
            }
            if(servoArgs.servoName.equals("horizontal")){
                horizontalPos = servoArgs.servoPos;
                hookPos = telescope.servos.getHook().getPosition();
                flipPos = telescope.servos.getFlip().getPosition();
            }
        }else{
            horizontalPos = telescope.servos.getHorizontal().getPosition();
            hookPos = telescope.servos.getHook().getPosition();
            flipPos = telescope.servos.getFlip().getPosition();
        }

        telescope.setTelescope(power, true, targetHeight, horizontalPos, hookPos, flipPos);

        if(telescope.lift.selfData.getCurHeight() == liftArgs.height
                && telescope.servos.getHorizontal().getPosition() == horizontalPos
                && telescope.servos.getHook().getPosition() == hookPos
                && telescope.servos.getFlip().getPosition() == flipPos  ||
                power == 0 && targetHeight == 0 && telescope.servos.getHorizontal().getPosition() == horizontalPos
                && telescope.servos.getHook().getPosition() == hookPos
                && telescope.servos.getFlip().getPosition() == flipPos ){
            isDone = true;
        }

        showData();
    }

    @Override
    public void showData() {
        telescope.lift.selfData.showHeight();
        telescope.servos.showServosPos();
    }
}
