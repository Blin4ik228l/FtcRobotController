package org.firstinspires.ftc.teamcode.Programms.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;

@Autonomous(name = "TestAuto", group = "Unknown")
public class AutoTest extends LinearOpModeModernized {
    @Override
    public void initialization() {
        generalInformation = new GeneralInformation(GeneralInformation.ProgramName.Auto, GeneralInformation.Color.Blue, GeneralInformation.StartPos.Nevermind);
    }

    @Override
    public void extRun() {

        while (!isStarted()){


            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()){


            telemetry.update();
        }
    }
}
