package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class ButtonClass extends UpdatableModule {
    public ButtonClass(OpMode op) {
        super(op);
        try {

        } catch (Exception e) {
            isInizialized = false;
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
    public void showData() {
        telemetry.addLine("===BUTTON CLASS===");
        telemetry.addData("State", curState);
        telemetry.addData("Dist", button.getDistance(DistanceUnit.CM));
        telemetry.addLine();
    }
}
