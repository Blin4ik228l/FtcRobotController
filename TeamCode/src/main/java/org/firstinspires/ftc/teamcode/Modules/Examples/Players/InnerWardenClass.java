package org.firstinspires.ftc.teamcode.Modules.Examples.Players;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutomaticClass;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class InnerWardenClass extends UpdatableModule {
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
    public void update() {
        robotClass.driveTrain.exOdometry.setPos(robotClass.driveTrain.cameraClass.getPos());

        robotClass.driveTrain.cameraClass.setFields(robotClass.driveTrain.exOdometry.encGlobalPosition2D, robotClass.driveTrain.exOdometry.encHeadVel,
                robotClass.driveTrain.exOdometry.robotCurVelocity.length());

        automaticClass.setFields(robotClass.driveTrain.exOdometry.teamColor.getRandomizedArtifact(), robotClass.driveTrain.exOdometry.isVyrCompleted,
                robotClass.driveTrain.exOdometry.getRange(), robotClass.driveTrain.exOdometry.encHeadVel, robotClass.driveTrain.exOdometry.robotCurVelocity.length());

        robotClass.driveTrain.motors.setKPower(robotClass.voltageSensor.kPower);
        robotClass.collector.motors.setKPower(robotClass.voltageSensor.kPower);
    }
}
