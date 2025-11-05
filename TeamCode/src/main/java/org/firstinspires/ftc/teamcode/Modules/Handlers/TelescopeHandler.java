package org.firstinspires.ftc.teamcode.Modules.Handlers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

public class TelescopeHandler extends Handler {
    public TelescopeHandler(RobotClass.Collector collector, Telemetry telemetry) {
        super(telemetry);
        this.collector = collector;
    }
    public void setArgs(Args.LiftArgs liftArgs){
        this.liftArgs = liftArgs;
    }
    public void setArgs(Args.ServoArgs servoArgs){
        this.servoArgs = servoArgs;
    }
    public RobotClass.Collector collector;
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

//        if(servoArgs != null){
//            if(servoArgs.servoName.equals("hook")){
//                hookPos = servoArgs.servoPos;
//                horizontalPos = collector.servos.getHorizontal().getPosition();
//                flipPos = collector.servos.getBaraban().getPosition();
//            }
//            if(servoArgs.servoName.equals("flip")){
//                flipPos = servoArgs.servoPos;
//                horizontalPos = collector.servos.getHorizontal().getPosition();
//                hookPos = collector.servos.getHook().getPosition();
//            }
//            if(servoArgs.servoName.equals("horizontal")){
//                horizontalPos = servoArgs.servoPos;
//                hookPos = collector.servos.getHook().getPosition();
//                flipPos = collector.servos.getBaraban().getPosition();
//            }
//        }else{
//            horizontalPos = collector.servos.getHorizontal().getPosition();
//            hookPos = collector.servos.getHook().getPosition();
//            flipPos = collector.servos.getBaraban().getPosition();
//        }
//
//        collector.setTelescope(power, true, targetHeight, horizontalPos, hookPos, flipPos);
//
//        if(collector.motors.selfData.getCurHeight() == liftArgs.height
//                && collector.servos.getHorizontal().getPosition() == horizontalPos
//                && collector.servos.getHook().getPosition() == hookPos
//                && collector.servos.getBaraban().getPosition() == flipPos  ||
//                power == 0 && targetHeight == 0 && collector.servos.getHorizontal().getPosition() == horizontalPos
//                && collector.servos.getHook().getPosition() == hookPos
//                && collector.servos.getBaraban().getPosition() == flipPos ){
//            isDone = true;
//        }

        showData();
    }

    @Override
    public void showData() {
        collector.servos.showServosPos();
    }
}
