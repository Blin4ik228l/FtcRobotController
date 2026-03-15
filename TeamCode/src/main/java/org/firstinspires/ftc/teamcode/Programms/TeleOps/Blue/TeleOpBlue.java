package org.firstinspires.ftc.teamcode.Programms.TeleOps.Blue;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.TeamAliance;
import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

@TeleOp(name = "Blue", group = "Blue")
public class TeleOpBlue extends TeleOpModernized {
    @Override
    public void init() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.TeleOp, TeamAliance.BLUE, GeneralInformation.StartPos.Nevermind);

        initAfterRobot();
    }

    @Override
    public void extUpdate() {

    }

    @Override
    public void extExecute() {

    }

    @Override
    public void extShow() {

    }
}
