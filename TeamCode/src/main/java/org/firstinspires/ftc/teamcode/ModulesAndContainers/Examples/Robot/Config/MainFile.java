package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;

public class MainFile {
    public static ExpansionHubDevices expansionHubDevices = new ExpansionHubDevices();
    public static ControlHubDevices controlHubDevices = new ControlHubDevices();

    public static OpMode op;
    public static VoltageSensorClass voltageSensorClass;
    public static GeneralInformation generalInformation;

    public MainFile(OpMode op, GeneralInformation generalInformation){
        this.generalInformation = generalInformation;
        this.op = op;
    }
    public void setVoltageSensorClass(VoltageSensorClass voltageSensorClass){
        this.voltageSensorClass = voltageSensorClass;
    }
}
