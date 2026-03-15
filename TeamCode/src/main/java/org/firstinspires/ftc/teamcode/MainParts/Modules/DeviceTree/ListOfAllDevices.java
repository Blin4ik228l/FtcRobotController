package org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.ControlHubDevices;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.ExpansionHubDevices;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Module;

public abstract class ListOfAllDevices extends Module {
    protected static ExpansionHubDevices expansionHubDevices;
    protected static ControlHubDevices controlHubDevices;
    public ListOfAllDevices() {
        controlHubDevices = MainFile.controlHubDevices;
        expansionHubDevices = MainFile.expansionHubDevices;
    }

}
