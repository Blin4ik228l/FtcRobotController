package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.ButtonClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.CollectorMotors;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.ColorSensorClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.ServomotorsClass;

public class Collector extends UpdatableModule {
    public CollectorMotors motors;
    public ServomotorsClass servos;
    public ColorSensorClass colorSensorClass;
    public DigitalCellsClass digitalCellsClass;
    public ButtonClass buttonClass;

    public Collector(OpMode op) {
        super(op);

        try {
            motors = new CollectorMotors(op);
        } catch (Exception e) {
            motors.isInizialized = false;
        }
        try {
            servos = new ServomotorsClass(op);
        } catch (Exception e) {
            servos.isInizialized = false;
        }

        try {
            colorSensorClass = new ColorSensorClass(op);
        } catch (Exception e) {
            colorSensorClass.isInizialized = false;
        }

        try {
            buttonClass = new ButtonClass(op);
        } catch (Exception e) {
            buttonClass.isInizialized = false;
        }

        try {
            digitalCellsClass = new DigitalCellsClass(servos, op);
        } catch (Exception e) {
            digitalCellsClass.isInizialized = false;
        }

        telemetry.addLine("Collector inited");
    }

    @Override
    public void update() {
        if(colorSensorClass.isInizialized) colorSensorClass.update();
    }

    @Override
    public void showData() {
        telemetry.addLine("===COLLECTOR DATA===");
        digitalCellsClass.showData();
        colorSensorClass.showData();
        servos.showData();
        motors.showData();
    }
}
