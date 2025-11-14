package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.AutomaticClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class InnerWardenClass extends Module{
    public RobotClass robotClass;
    public AutomaticClass automaticClass;
    public PlayerClass1 playerClass1;
    public InnerWardenClass(RobotClass robotClass, PlayerClass1 playerClass1, AutomaticClass automaticClass, OpMode op) {
        super(op.telemetry);
        this.robotClass = robotClass;
        this.playerClass1 = playerClass1;
        this.automaticClass = automaticClass;
    }

    @Override
    public void execute() {
        automaticClass.setRandomizedArtifacts(robotClass.cameraClass.randomizedArtifact);
        automaticClass.setFields(robotClass.driveTrain.exOdometry.isVyrCompleted, robotClass.driveTrain.exOdometry.getRange(), robotClass.driveTrain.exOdometry.robotSelfCentricVel.length());
    }
}
