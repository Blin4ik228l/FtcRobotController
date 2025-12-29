package org.firstinspires.ftc.teamcode.Robot.RobotParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;

public class TelemetrySettings extends UpdatableModule {
    public TelemetrySettings(TeleOpModernized teleOpModernized, LinearOpModeModernized linearOpModeModernized) {
        super(teleOpModernized.telemetry);

        this.teleOpModernized = teleOpModernized;
        this.linearOpModeModernized = linearOpModeModernized;

        if(linearOpModeModernized == null) opMode = OpMode.Tele;
        else opMode = OpMode.Auto;

        telemetryMode = TelemetryMode.Show_all;
    }
    public TelemetryMode telemetryMode;
    public TeleOpModernized teleOpModernized;
    public LinearOpModeModernized linearOpModeModernized;
    public OpMode opMode;
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
    public enum OpMode{
        Auto,
        Tele
    }

    @Override
    public void update() {

        if(opMode == OpMode.Tele) {
            teleOpModernized.joystickActivityClass.tA_and_Left_Trigger %= 9;

            switch (teleOpModernized.joystickActivityClass.tA_and_Left_Trigger) {
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

        switch (telemetryMode){
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
                teleOpModernized.robot.driveTrain.motors.showData();
                teleOpModernized.robot.collector.motors.showData();
                break;
            case Show_voltage:
                teleOpModernized.robot.voltageSensor.showData();
                break;
            case Show_navigation:
                teleOpModernized.robot.driveTrain.positionRobotController.showData();
                teleOpModernized.robot.driveTrain.cameraClass.showData();
                teleOpModernized.robot.driveTrain.odometryClass.showData();
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
                break;
            case Show_nothing:
                break;
        }
    }
}
