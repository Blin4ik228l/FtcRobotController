package org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.ButtonClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.CollectorMotors;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.ColorSensorClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.DigitalCellsClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts.ServomotorsClass;

public class Collector extends UpdatableModule {
    public CollectorMotors motors;
    public ServomotorsClass servos;
    public ColorSensorClass colorSensorClass;
    public DigitalCellsClass digitalCellsClass;
    public ButtonClass buttonClass;

    public Collector(OpMode op) {
        super(op);

        motors = new CollectorMotors(op);

        servos = new ServomotorsClass(op);
        colorSensorClass = new ColorSensorClass(op);
        buttonClass = new ButtonClass(op);

        digitalCellsClass = new DigitalCellsClass(servos, op);

        telemetry.addLine("Collector inited");
    }

    @Override
    public void setIteration(int iterationCount) {
        super.setIteration(iterationCount);
        motors.setIteration(iterationCount);
        servos.setIteration(iterationCount);
        colorSensorClass.setIteration(iterationCount);
        buttonClass.setIteration(iterationCount);
        digitalCellsClass.setIteration(iterationCount);
    }

    @Override
    public void resetTimer() {
        innerRunTime.reset();
        motors.resetTimer();
        servos.resetTimer();
        colorSensorClass.resetTimer();
        buttonClass.resetTimer();
        digitalCellsClass.resetTimer();
    }

    @Override
    public void update() {
        colorSensorClass.update();
    }

    @Override
    public void showData() {
        digitalCellsClass.showData();
        colorSensorClass.showData();
        servos.showData();
        motors.showData();
    }
}
