package org.firstinspires.ftc.teamcode.Robot.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;

public class TelemetrySettings extends UpdatableModule {
    public TelemetrySettings(TeleOpModernized teleOpModernized, LinearOpModeModernized linearOpModeModernized, OpMode op) {
        super(op);

        this.teleOpModernized = teleOpModernized;
        this.linearOpModeModernized = linearOpModeModernized;

        if(linearOpModeModernized == null) opMode = Mode.Tele;
        else opMode = Mode.Auto;

        telemetryMode = TelemetryMode.Show_all;
    }
    public TelemetryMode telemetryMode;
    public TeleOpModernized teleOpModernized;
    public LinearOpModeModernized linearOpModeModernized;
    public Mode opMode;
    public enum TelemetryMode{
        Show_all,
        Show_joysticks,
        Show_navigation,
        Show_voltage,
        Show_motors,
        Show_sorting_system,
        Show_players,
        Show_other,
        Show_nothing,

        Show_for_auto
    }
    public enum Mode {
        Auto,
        Tele
    }

    @Override
    public void update() {

        if(opMode == Mode.Tele) {
            teleOpModernized.joystickActivityClass.tBPressed %= 9;

            switch (teleOpModernized.joystickActivityClass.tBPressed) {
                case 0:
                    telemetryMode = TelemetryMode.Show_all;
                    break;
                case 1:
                    telemetryMode = TelemetryMode.Show_joysticks;
                    break;
                case 2:
                    telemetryMode = TelemetryMode.Show_navigation;
                    break;
                case 3:
                    telemetryMode = TelemetryMode.Show_sorting_system;
                    break;
                case 4:
                    telemetryMode = TelemetryMode.Show_motors;
                    break;
                case 5:
                    telemetryMode = TelemetryMode.Show_voltage;
                    break;
                case 6:
                    telemetryMode = TelemetryMode.Show_players;
                    break;
                case 7:
                    telemetryMode = TelemetryMode.Show_other;
                    break;
                case 8:
                    telemetryMode = TelemetryMode.Show_nothing;
                    break;
            }
        }else telemetryMode = TelemetryMode.Show_for_auto;
    }

    @Override
    public void showData() {
        telemetry.addLine(telemetryMode.toString());

        switch (telemetryMode) {
            case Show_all:
                teleOpModernized.joystickActivityClass.showData();
                teleOpModernized.joystickActivityClass2.showData();

                teleOpModernized.player1.showData();
                teleOpModernized.autoPlayerClass.showData();

                teleOpModernized.robot.showData();
                break;
            case Show_joysticks:
                teleOpModernized.joystickActivityClass.showData();
                teleOpModernized.joystickActivityClass2.showData();
                break;
            case Show_motors:
                teleOpModernized.robot.drivetrain.motors.showData();
                teleOpModernized.robot.collector.motors.showData();
                break;
            case Show_voltage:
                teleOpModernized.robot.voltageSensor.showData();
                break;
            case Show_navigation:
                teleOpModernized.robot.drivetrain.positionRobotController.showData();
                teleOpModernized.robot.drivetrain.cameraClass.showData();
                teleOpModernized.robot.drivetrain.odometryClass.showData();
                break;
            case Show_sorting_system:
                teleOpModernized.robot.collector.colorSensorClass.showData();
                teleOpModernized.robot.collector.digitalCellsClass.showData();
                break;
            case Show_players:
                teleOpModernized.player1.showData();
                teleOpModernized.autoPlayerClass.showData();
                break;
            case Show_other:
                teleOpModernized.extShow();
                teleOpModernized.robot.collector.motors.showData();
                teleOpModernized.autoPlayerClass.showData();
                teleOpModernized.robot.collector.servos.showData();
                break;
            case Show_nothing:
                break;
            case Show_for_auto:
                linearOpModeModernized.driveHandler.showData();

                linearOpModeModernized.robot.drivetrain.odometryClass.showData();
                linearOpModeModernized.robot.drivetrain.motors.showData();
                break;
        }

    }
}
