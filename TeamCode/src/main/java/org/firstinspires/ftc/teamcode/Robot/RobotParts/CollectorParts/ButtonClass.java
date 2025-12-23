package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Types.Module;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;

public class ButtonClass extends UpdatableModule {
    public ButtonClass(OpMode op) {
        super(op.telemetry);
        button = op.hardwareMap.get(DistanceSensor.class, "dist");

        telemetry.addLine("Button is Inited");
    }
    public DistanceSensor button;
    public State curState = State.Unready;
    public enum State{
        Ready,
        Unready
    }

    @Override
    public void update() {
        if(button.getDistance(DistanceUnit.CM) < 6 && button.getDistance(DistanceUnit.CM) > 3.2) curState = State.Ready;
        else curState = State.Unready;
    }


    @Override
    public void showData() {
        telemetry.addLine("===BUTTON CLASS===");
        telemetry.addData("State", curState);
        telemetry.addData("Dist", button.getDistance(DistanceUnit.CM));
        telemetry.addLine();
    }
}
