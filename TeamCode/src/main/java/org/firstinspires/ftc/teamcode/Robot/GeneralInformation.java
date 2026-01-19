package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;

public class GeneralInformation {
    public StartPos startPos;
    public Color color;
    public ProgramName programName;
    public GeneralInformation(ProgramName programName, Color color, StartPos startPos){
        this.programName = programName;
        this.color = color;
        this.startPos = startPos;
    }

    public enum Color{
        Red,
        Blue
    }
    public enum StartPos{
        Near_wall,
        Far_from_wall,
        Nevermind
    }
    public enum ProgramName{
        Auto,
        TeleOp
    }
}
