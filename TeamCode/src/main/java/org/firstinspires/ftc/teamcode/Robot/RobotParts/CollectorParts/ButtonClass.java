package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;

public class ButtonClass extends Module {
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
    public boolean state;

    public boolean getState(){
//        state = button.getState();
        return false;
    }


    @Override
    public void showData() {
        telemetry.addLine("===BUTTON CLASS===");
        telemetry.addData("State", state);
        telemetry.addLine();
    }
}
