package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

import java.util.ArrayList;

public class DigitalCellsClass extends UpdatableModule {
    public CellWrapper.Builder cells;
    public DigitalCellsClass(OpMode op){
        super(op);

        cells = new CellWrapper.Builder()
                .add("1",
                        new ColorSensorWrapper.Builder()
                                .initialize(op, "color1")
                                .setFields("color1", new double[]{})
                                .initialize(op, "color2")
                                .setFields("color2", new double[]{}),
                        new ServoMotorWrapper(op, "servo1"))
                .add("2",
                        new ColorSensorWrapper.Builder()
                                .initialize(op, "color3")
                                .setFields("color3", new double[]{})
                                .initialize(op, "color4")
                                .setFields("color4", new double[]{}),
                        new ServoMotorWrapper(op, "servo2"))
                .add("3",
                        new ColorSensorWrapper.Builder()
                                .initialize(op, "color5")
                                .setFields("color5", new double[]{})
                                .initialize(op, "color6")
                                .setFields("color6", new double[]{}),
                        new ServoMotorWrapper(op, "servo3"));

    }
    /*1 - в массиве это зелёный шар
     * 2 - в массиве это фиолетовый*/
    private int artifactCount;
    public boolean isInerrupted;
    public ServoMotorWrapper triggeredServo;
    public int getArtifactCount() {
        return artifactCount;
    }

    @Override
    public void update() {
        artifactCount = cells.updateAll().numberOfArtifacts();
    }

    public void fire(int color){
        if(isInerrupted) return;
        CellWrapper founded = cells.getNeededCell(color);

        triggeredServo = founded.servoWrapper;
        isInerrupted = true;

        switch (founded.name){
            case "1":
                founded.servoWrapper.setSignal(1);
            case "2":
                founded.servoWrapper.setSignal(1);
            default:
                founded.servoWrapper.setSignal(1);
        }

    }
    public void prepareServo(){
       cells.getCell("1").servoWrapper.setPosition(0);
       cells.getCell("2").servoWrapper.setPosition(0);
       cells.getCell("3").servoWrapper.setPosition(0);
    }

    @Override
    public void showData() {
        telemetry.addLine("===DIGITAL CELLS===");
        telemetry.addData("Count", artifactCount);
        telemetry.addData("Cells", "C0:%s C1:%s C2:%s", "");
        telemetry.addLine();
    }

    public static class CellWrapper{
        public ColorSensorWrapper.Builder sensorsWrapper;
        public ServoMotorWrapper servoWrapper;
        public CellWrapper(String name, ColorSensorWrapper.Builder sensorsWrapper, ServoMotorWrapper servoWrapper){
            this.sensorsWrapper = sensorsWrapper;
            this.servoWrapper = servoWrapper;
            this.name = name;
        }
        public String name;
        public String getColorFromNumber(double number){
            return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
        }

        public static class Builder{
            private ArrayList<CellWrapper> cells = new ArrayList<>();

            public Builder add(String cellName, ColorSensorWrapper.Builder sensors, ServoMotorWrapper servo){
                cells.add(new CellWrapper(cellName, sensors, servo));
                return this;
            }
            public Builder updateAll(){
                for (CellWrapper cell : cells) {
                    cell.sensorsWrapper.updateAll();
                }
                return this;
            }
            public int numberOfArtifacts(){
                int artifactNumber = 0;
                for (CellWrapper cell : cells) {
                    artifactNumber += cell.sensorsWrapper.foundedColor() != 0 ? 1 : 0;
                }
                return artifactNumber;
            }
            public CellWrapper getNeededCell(int color){
                CellWrapper reserveCell = cells.get(0);
                for (CellWrapper cell : cells) {
                    if (cell.sensorsWrapper.foundedColor() != 0) reserveCell = cell;
                    if (cell.sensorsWrapper.foundedColor() == color) return cell;
                }
                return reserveCell;
            }
            public CellWrapper getEmptyCell(){
                CellWrapper reserveCell = cells.get(0);
                for (CellWrapper cell : cells) {
                   if (cell.sensorsWrapper.foundedColor() == 0) return cell;
                }
                return reserveCell;
            }
            public CellWrapper getCell(String name){
                CellWrapper reserveCell = cells.get(0);
                for (CellWrapper cell : cells) {
                    if(cell.name == name) return reserveCell;
                }
                return reserveCell;
            }
        }
    }
}
