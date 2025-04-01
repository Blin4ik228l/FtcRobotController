package org.firstinspires.ftc.teamcode.Game.Robot.Modules.Unused;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Module;

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
