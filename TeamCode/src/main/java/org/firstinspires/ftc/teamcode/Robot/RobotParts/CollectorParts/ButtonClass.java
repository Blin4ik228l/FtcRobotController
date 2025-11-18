package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;

public class ButtonClass extends UpdatableModule {
    public ButtonClass(OpMode op) {
        super(op.telemetry);
//        button = op.hardwareMap.get(DigitalChannel.class, "digitalTouch");
//
//        button.setMode(DigitalChannel.Mode.INPUT);
//
//        buttonClassState = ButtonClassState.upDateState;
        telemetry.addLine("Button is Inited");
    }
    public DigitalChannel button;
    public int count;
    public ButtonClassState buttonClassState;
    public enum ButtonClassState{
        wasPressed,
        upDateState,
        noPressed
    }

    @Override
    public void update() {
//        if(!button.getState() && count == 0) buttonClassState = ButtonClassState.noPressed;
//
//        if(button.getState() && count == 0) {
//            count++;
//            buttonClassState = ButtonClassState.wasPressed;}
//
//        if (count != 0 && buttonClassState == ButtonClassState.upDateState) {
//            count = 0;
//        }
    }

    @Override
    public void showData() {
        telemetry.addLine("===BUTTON CLASS===");
        telemetry.addData("State", buttonClassState.toString());
        telemetry.addLine();
    }
}
