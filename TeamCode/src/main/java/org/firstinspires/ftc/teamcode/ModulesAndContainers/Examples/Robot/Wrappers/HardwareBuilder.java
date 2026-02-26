package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class HardwareBuilder <T> {
    public abstract  T initialize(OpMode op, String deviceName);
}
