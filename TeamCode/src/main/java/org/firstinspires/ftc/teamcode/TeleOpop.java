package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Meow", group = "huy")
public class TeleOpop extends OpMode {
InitClass initClass = new InitClass();
    Telemetry telemetry;
    @Override
    public void init() {
        initClass.initDevices();
    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {
        super.start();
        telemetry.addLine("Robot Started");
    }
}
