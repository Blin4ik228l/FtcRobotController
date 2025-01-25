package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

public class Button implements Module {
    private OpMode op;

    private RevTouchSensor touch;

    private boolean switchTouch = false, isTouchedBol;
    private int timesTouched;

   public Button(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        touch = op.hardwareMap.get(RevTouchSensor.class, "touch");
    }

    public int getTimesTouched() {
        return timesTouched;
    }

    public boolean isTouched(){
       if(touch.isPressed() && !switchTouch){
           timesTouched ++;
           isTouchedBol = true;
           switchTouch = true;
       }
       return isTouchedBol;
    }

}
