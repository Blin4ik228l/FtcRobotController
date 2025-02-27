package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Singles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;

public class Camera implements Module {
    private final OpMode op;

    private WebcamName webcam1;

    public Camera (OpMode op){
        this.op = op;
    }
    @Override
    public void init() {
        webcam1 = op.hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    //TODO: Осуществить работу с камерой
}
