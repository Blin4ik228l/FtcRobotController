package org.firstinspires.ftc.teamcode.Modules.Types;

import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;

@FunctionalInterface
public interface InterSetGen {
    void setGeneralInformation(GeneralInformation.ProgramName programName, GeneralInformation.Color color, GeneralInformation.StartPos startPos);
}
