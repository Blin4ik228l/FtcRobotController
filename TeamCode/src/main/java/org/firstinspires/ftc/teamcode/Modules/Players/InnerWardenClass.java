package org.firstinspires.ftc.teamcode.Modules.Players;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.ExecutableModule;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.AutomaticClass;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;
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
        robotClass.driveTrain.exOdometry.setPosFromCamera(robotClass.cameraClass.returnWritedPos());

        automaticClass.setRandomizedArtifacts(robotClass.cameraClass.randomizedArtifact);

        robotClass.cameraClass.setOdometryState(robotClass.driveTrain.exOdometry.isPosFromCameraWasGotFirstly);

        robotClass.cameraClass.setPositionFromOdometry(robotClass.driveTrain.exOdometry.encGlobalPosition2D);
        robotClass.cameraClass.setRobotVelFromOdometry(robotClass.driveTrain.exOdometry.robotCurVelocity, robotClass.driveTrain.exOdometry.encHeadVel);
        robotClass.cameraClass.setRangeFromOdometry(robotClass.driveTrain.exOdometry.getRange());

        automaticClass.setFields(robotClass.driveTrain.exOdometry.isVyrCompleted, robotClass.driveTrain.exOdometry.getRange(), robotClass.driveTrain.exOdometry.robotCurVelocity.length());
    }
}
