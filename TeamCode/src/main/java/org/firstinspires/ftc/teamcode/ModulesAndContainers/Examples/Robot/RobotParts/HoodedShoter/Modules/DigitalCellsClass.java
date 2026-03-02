package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

import java.util.ArrayList;

public class DigitalCellsClass extends UpdatingModule {
    public CellWrapper.Builder cells;
    public DigitalCellsClass(MainFile mainFile){
        super(mainFile);

        createColorWrapperUtils();
        createServoWrapperUtils();
        cells = new CellWrapper.Builder()
                .add(mainFile,"cell1",
                        servoBuilder.initialize(mainFile, controlHubDevices.getServo0(0)).setFields(60.0, 270.0).get(),
                        colorBuilder.initialize(mainFile, expansionHubDevices.getI2C(0)).setFields(0.0, 0.0, 0.0, 0.0).get(),
                        colorBuilder.initialize(mainFile, expansionHubDevices.getI2C(1)).setFields(0.0, 0.0, 0.0, 0.0).get()
                )
                .add(mainFile,"cell2",
                        servoBuilder.initialize(mainFile, expansionHubDevices.getServo0(1)).setFields(60.0, 270.0).get(),
                        colorBuilder.initialize(mainFile, expansionHubDevices.getI2C(2)).setFields(0.0, 0.0, 0.0, 0.0).get(),
                        colorBuilder.initialize(mainFile, expansionHubDevices.getI2C(3)).setFields(0.0, 0.0, 0.0, 0.0).get())
                .add(mainFile,"cell3",
                        servoBuilder.initialize(mainFile, expansionHubDevices.getServo0(2)).setFields(60.0, 270.0).get(),
                        colorBuilder.initialize(mainFile, controlHubDevices.getI2C(0)).setFields(0.0, 0.0, 0.0, 0.0).get(),
                        colorBuilder.initialize(mainFile, controlHubDevices.getI2C(1)).setFields(0.0, 0.0, 0.0, 0.0).get());


        sayCreated();
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
    protected void updateExt() {
        artifactCount = cells.updateAll().numberOfArtifacts();
    }

    @Override
    protected void showDataExt() {
        telemetry.addData("Count", artifactCount);
        cells.showAll();
        telemetry.addLine();
    }


    public void fire(int color){
        if(isInerrupted) return;
        CellWrapper founded = cells.getNeededCell(color);

        triggeredServo = founded.servoWrapper;
        isInerrupted = true;

        switch (founded.name){
            case "cell1":
                founded.servoWrapper.execute(1.0);
            case "cell2":
                founded.servoWrapper.execute(1.0);
            default:
                founded.servoWrapper.execute(1.0);
        }

    }
    public void prepareServo(){
       cells.getCell("1").servoWrapper.execute(0.0);
       cells.getCell("2").servoWrapper.execute(0.0);
       cells.getCell("3").servoWrapper.execute(0.0);
    }

    public static class CellWrapper extends UpdatingModule {
        public ArrayList<ColorSensorWrapper> sensorsWrapper = new ArrayList<>();
        public ServoMotorWrapper servoWrapper;
        public boolean isInit;
        public String name;
        public CellWrapper(MainFile mainFile, String cellName, ServoMotorWrapper servoWrapper, ColorSensorWrapper...sensorsWrapperIn){
            super(mainFile);
            this.servoWrapper = servoWrapper;
            this.name = cellName;

            for (ColorSensorWrapper color:sensorsWrapperIn) {
                sensorsWrapper.add(color);
            };
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

        public String getColorFromNumber(double number){
            return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
        }

        @Override
        protected void updateExt() {
            for (ColorSensorWrapper color:sensorsWrapper) {
                color.update();
            };
        }

        @Override
        protected void sayModuleName() {
            telemetry.addLine(name.toUpperCase());
        }

        @Override
        protected void showDataExt() {
            servoWrapper.showData();
            for (ColorSensorWrapper color:sensorsWrapper) {
                color.showData();
            };
        }

        public static class Builder{
            private ArrayList<CellWrapper> cells = new ArrayList<>();

            public Builder add(MainFile mainFile, String cellName, ServoMotorWrapper servo, ColorSensorWrapper...sensors){
                cells.add(new CellWrapper(mainFile, cellName, servo, sensors));
                return this;
            }
            public Builder updateAll(){
                for (CellWrapper cell : cells) {
                    cell.update();
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
