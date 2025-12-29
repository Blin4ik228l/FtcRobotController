package org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class InnerWardenClass extends UpdatableModule {
    public RobotClass robotClass;
    public AutoPlayerClass autoPlayerClass;
    public PlayerClass1 playerClass1;
    public InnerWardenClass(RobotClass robotClass, PlayerClass1 playerClass1, AutoPlayerClass autoPlayerClass, OpMode op) {
        super(op.telemetry);
        this.robotClass = robotClass;
        this.playerClass1 = playerClass1;
        this.autoPlayerClass = autoPlayerClass;
    }

    @Override
    public void update() {
        robotClass.driveTrain.motors.setKPower(robotClass.voltageSensor.getkPower());
        robotClass.collector.motors.setKPower(robotClass.voltageSensor.getkPower());

        robotClass.collector.digitalCellsClass.setRandomizedArtifact(robotClass.driveTrain.cameraClass.getRandomizedArtifacts());

        autoPlayerClass.setFields(robotClass.driveTrain.cameraClass.randomizeStatus,
                robotClass.driveTrain.positionRobotController.vyrState,
                robotClass.driveTrain.odometryClass.moveState,
                robotClass.driveTrain.odometryClass.rotateState,
                robotClass.driveTrain.positionRobotController.getRange());
    }
}
