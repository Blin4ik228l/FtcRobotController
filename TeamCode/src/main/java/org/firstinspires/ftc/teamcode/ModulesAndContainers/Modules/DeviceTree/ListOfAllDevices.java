package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.DeviceTree;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.ControlHubDevices;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.ExpansionHubDevices;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

public abstract class ListOfAllDevices extends Module {
    protected static ExpansionHubDevices expansionHubDevices;
    protected static ControlHubDevices controlHubDevices;
    public ListOfAllDevices(MainFile mainFile) {
        super(mainFile);
        controlHubDevices = mainFile.controlHubDevices;
        expansionHubDevices = mainFile.expansionHubDevices;
    }

}
