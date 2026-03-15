package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.ServoMotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class DigitalCellsClass extends UpdatableCollector {
    public CellWrapper.Builder cells;

    public String pusher0 = controlHubDevices.getServo(0);
    public String pusher1 = controlHubDevices.getServo(1);
    public String pusher2 = controlHubDevices.getServo(2);
    public DigitalCellsClass(){
        super(false);

        createColorWrapperUtils();
        createServoWrapperUtils();
        cells = new CellWrapper.Builder()
                .add("cell1",
                        colorBuilder.initialize( expansionHubDevices.getI2C(0)).setFields(0.229, 0.297, 0.297, 0.0).get(),
                        colorBuilder.initialize( expansionHubDevices.getI2C(1)).setFields(0.229, 0.263, 0.263, 0.0).get()
                )
                .add("cell2",
                        colorBuilder.initialize( expansionHubDevices.getI2C(2)).setFields(0.186, 0.203, 0.203, 0.0).get(),
                        colorBuilder.initialize( expansionHubDevices.getI2C(3)).setFields(0.150, 0.203, 0.203, 0.0).get())
                .add("cell3",
                        colorBuilder.initialize( controlHubDevices.getI2C(0)).setFields(0.161, 0.195, 0.195, 0.0).get(),
                        colorBuilder.initialize( controlHubDevices.getI2C(1)).setFields(0.161, 0.186, 0.195, 0.0).get());

        servosCollector
                .add(servoBuilder.initialize( pusher0).setFields(1000.0, 180.0).get())
                .add(servoBuilder.initialize( pusher2).setFields(750.0, 180.0).get())
                .add(servoBuilder.initialize( pusher1).setFields(1000.0, 180.0).get());

        prepareServo();
        sayCreated();
    }
    /*1 - в массиве это зелёный шар
     * 2 - в массиве это фиолетовый*/
    private int artifactCount;
    public boolean isStopped;
    public ServoMotorWrapper triggeredServo;
    public CellWrapper triggeredCell;

    public int getArtifactCount() {
        return artifactCount;
    }
    @Override
    protected void updateExt() {
        artifactCount = cells.updateAll().numberOfArtifacts();
    }

    @Override
    protected void showDataExt() {
        if (triggeredCell != null) telemetry.addData("cell", triggeredCell.name);

        telemetry.addData("Count", artifactCount);
        cells.showAll();
        servosCollector.showData();
    }


    public void fire(int color){
        if(isStopped) return;
        triggeredCell = cells.getNeededCell(color);

        if (triggeredCell == null) {
            return;
        }
        switch (triggeredCell.name){
            case "cell1":
                servosCollector.get(pusher0).execute(0.52);
                triggeredServo = servosCollector.get(pusher0);
                break;
            case "cell2":
                servosCollector.get(pusher2).execute(0.48);
                triggeredServo = servosCollector.get(pusher2);
                break;
            case "cell3":
                servosCollector.get(pusher1).execute(0.5);
                triggeredServo = servosCollector.get(pusher1);
                break;
        }
        isStopped = true;
    }
    public void prepareServo(){
       servosCollector.get(pusher0).execute(0.08);
       servosCollector.get(pusher1).execute(0.07);
       servosCollector.get(pusher2).execute(0.08);
    }

    public static class CellWrapper extends UpdatableModule {
        public ArrayList<ColorSensorWrapper> sensorsWrapper = new ArrayList<>();
        public boolean isInit;
        public String name;
        public CellWrapper(String cellName, ColorSensorWrapper...sensorsWrapperIn){
            super(cellName);
            this.name = cellName;

            sensorsWrapper.addAll(Arrays.asList(sensorsWrapperIn));
        }
        public int isFounded(){
            int count = 0;
            for (ColorSensorWrapper color : sensorsWrapper) {
                if(color.getFoundedColor() != 0) count |= 1;
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
        public void sayModuleName() {
            telemetry.addLine(name.toUpperCase());
        }

        @Override
        protected void updateExt() {
            for (ColorSensorWrapper color:sensorsWrapper) {
                color.update();
            };
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("color", getColor());
//            for (ColorSensorWrapper color:sensorsWrapper) {
//                color.showData();
//            };
        }

        public static class Builder{
            private HashMap<String, CellWrapper> cells = new HashMap<>();

            public Builder add(String cellName, ColorSensorWrapper...sensors){
                cells.put(cellName, new CellWrapper(cellName, sensors));
                return this;
            }
            public Builder updateAll(){
                for (CellWrapper cell : cells.values()) {
                    cell.update();
                }
                return this;
            }
            public int numberOfArtifacts(){
                int artifactNumber = 0;
                for (CellWrapper cell : cells.values()) {
                    artifactNumber += cell.isFounded();
                }
                return artifactNumber;
            }
            public CellWrapper getNeededCell(int color){
                CellWrapper found = null;
                for (CellWrapper cell : cells.values()) {
                    if (cell.getColor() == color) {found = cell; break;}
                    else found = cell;
                }
                return found;
            }
            public CellWrapper getEmptyCell(){
                CellWrapper reserveCell = cells.get(0);
                for (CellWrapper cell : cells.values()) {
                   if (cell.getColor() == 0) return cell;
                }
                return reserveCell;
            }
            public boolean isInited(){
                boolean isInited = true;
                for (CellWrapper cell: cells.values()) {
                    isInited &= cell.isInit;
                }
                return isInited;
            }
            public CellWrapper getCell(String name){
                return cells.get(name);
            }
            public void showAll(){
                for (CellWrapper cell : cells.values()) {
                    cell.showData();
                }
            }
        }
    }
}
