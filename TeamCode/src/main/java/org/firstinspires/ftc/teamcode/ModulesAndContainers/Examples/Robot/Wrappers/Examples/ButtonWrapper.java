package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Extenders.DeviceUpdaterWrapper;

public class ButtonWrapper extends DeviceUpdaterWrapper {
    public ButtonWrapper(OpMode op) {
        super(op);
        try {

        } catch (Exception e) {
            isInitialized = false;
            return;
        }
        button = hardwareMap.get(DistanceSensor.class, "dist");

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
        double dist = button.getDistance(DistanceUnit.CM);

        if(dist < 6 && dist > 3.2) curState = State.Ready;
        else curState = State.Unready;
    }

    @Override
    protected void updateExt() {

    }


    @Override
    public void showData() {
        telemetry.addLine("===BUTTON CLASS===");
        telemetry.addData("State", curState);
        telemetry.addData("Dist", button.getDistance(DistanceUnit.CM));
        telemetry.addLine();
    }

    @Override
    public void showDataExt() {

    }
}
