package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.ArrayList;

public class DigitalCellsClass extends UpdatableModule {
    private ServoMotorWrapper.Builder servoBuilder = new ServoMotorWrapper.Builder();
    private ColorSensorWrapper.Builder colorBuilder = new ColorSensorWrapper.Builder();
    public CellWrapper.Builder cells;
    public DigitalCellsClass(OpMode op){
        super(op);

        cells = new CellWrapper.Builder()
                .add(op,"cell1",
                        servoBuilder.initialize(op, controlHubDevices.getServo0(0)).setFields(60, 270).get(),
                        colorBuilder.initialize(op, controlHubDevices.getI2C(2)).setFields(new double[]{}).get(),
                        colorBuilder.initialize(op, controlHubDevices.getI2C(3)).setFields(new double[]{}).get()
                )
                .add(op,"cell2",
                        servoBuilder.initialize(op, controlHubDevices.getServo0(1)).setFields(60, 270).get(),
                        colorBuilder.initialize(op, controlHubDevices.getI2C(1)).setFields(new double[]{}).get(),
                        colorBuilder.initialize(op, controlHubDevices.getI2C(0)).setFields(new double[]{}).get())
                .add(op,"cell3",
                        servoBuilder.initialize(op, controlHubDevices.getServo0(2)).setFields(60, 270).get(),
                        colorBuilder.initialize(op, expansionHubDevices.getI2C(1)).setFields(new double[]{}).get(),
                        colorBuilder.initialize(op, expansionHubDevices.getI2C(1)).setFields(new double[]{}).get());


        this.isInitialized = cells.isInited();
        sayInited();
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
            case "cell1":
                founded.servoWrapper.setSignal(1);
            case "cell2":
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
        sayModuleName();
        telemetry.addData("Count", artifactCount);
        cells.showAll();
        telemetry.addLine();
    }

    public static class CellWrapper extends Module {
        public ArrayList<ColorSensorWrapper> sensorsWrapper;
        public ServoMotorWrapper servoWrapper;
        public boolean isInit;
        public String name;
        public CellWrapper(OpMode op,String cellName, ServoMotorWrapper servoWrapper, ColorSensorWrapper...sensorsWrapperIn){
            super(op);
            this.servoWrapper = servoWrapper;
            this.name = cellName;

            for (ColorSensorWrapper color:sensorsWrapperIn) {
                isInit &= color.isInitialized;
                sensorsWrapper.add(color);
            };

            this.isInit &= this.servoWrapper.isInitialized;
        }
        public int isFounded(){
            int count = 0;
            for (ColorSensorWrapper color:sensorsWrapper) {
                if(color.getFoundedColor() != 0) count = 1; break;
            };
            return count;
        }
        public int getColor(){
            int clr = 0;
            for (ColorSensorWrapper color:sensorsWrapper) {
                clr = color.getFoundedColor();
                if(clr != 0) break;
            };
            return clr;
        }
        public void updateAll(){
            for (ColorSensorWrapper color:sensorsWrapper) {
                color.update();
            };
        }

        public String getColorFromNumber(double number){
            return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
        }
        public void showData(){
            telemetry.addLine(name);
            servoWrapper.showData();
            for (ColorSensorWrapper color:sensorsWrapper) {
                color.showData();
            };
        }

        public static class Builder{
            private ArrayList<CellWrapper> cells = new ArrayList<>();

            public Builder add(OpMode op, String cellName, ServoMotorWrapper servo, ColorSensorWrapper...sensors){
                cells.add(new CellWrapper(op, cellName, servo, sensors));
                return this;
            }
            public Builder updateAll(){
                for (CellWrapper cell : cells) {
                    cell.updateAll();
                }
                return this;
            }
            public int numberOfArtifacts(){
                int artifactNumber = 0;
                for (CellWrapper cell : cells) {
                    artifactNumber += cell.isFounded();
                }
                return artifactNumber;
            }
            public CellWrapper getNeededCell(int color){
                CellWrapper reserveCell = cells.get(0);
                for (CellWrapper cell : cells) {
                    if (cell.getColor() != 0) reserveCell = cell;
                    if (cell.getColor() == color) return cell;
                }
                return reserveCell;
            }
            public CellWrapper getEmptyCell(){
                CellWrapper reserveCell = cells.get(0);
                for (CellWrapper cell : cells) {
                   if (cell.getColor() == 0) return cell;
                }
                return reserveCell;
            }
            public boolean isInited(){
                boolean isInited = true;
                for (CellWrapper cell:cells) {
                    isInited &= cell.isInit;
                }
                return isInited;
            }
            public CellWrapper getCell(String name){
                CellWrapper reserveCell = cells.get(0);
                for (CellWrapper cell : cells) {
                    if(cell.name == name) return reserveCell;
                }
                return reserveCell;
            }
            public void showAll(){
                for (CellWrapper cell : cells) {
                    cell.showData();
                }
            }
        }
    }
}
