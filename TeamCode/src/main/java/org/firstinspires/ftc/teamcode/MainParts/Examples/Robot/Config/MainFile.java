package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.VoltageSensorClass;

public class MainFile {
    public static ExpansionHubDevices expansionHubDevices = new ExpansionHubDevices();
    public static ControlHubDevices controlHubDevices = new ControlHubDevices();

    public static OpMode op;
    public static VoltageSensorClass voltageSensorClass;
    public static GeneralInformation generalInformation;

    public MainFile(OpMode op, GeneralInformation generalInformation){
        MainFile.generalInformation = generalInformation;
        MainFile.op = op;
    }
    public void setVoltageSensorClass(VoltageSensorClass voltageSensorClass){
        MainFile.voltageSensorClass = voltageSensorClass;
    }
}
