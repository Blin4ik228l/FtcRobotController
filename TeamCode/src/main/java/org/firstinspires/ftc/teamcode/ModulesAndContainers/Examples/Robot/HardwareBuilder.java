package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashMap;

public abstract class HardwareBuilder <T> {
    public abstract  T initialize(OpMode op, String deviceName);
}
