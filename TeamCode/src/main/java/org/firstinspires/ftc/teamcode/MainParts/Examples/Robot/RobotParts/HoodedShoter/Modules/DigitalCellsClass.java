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
        double tresh = 1.6;
        double treshG = 2.2;
//        cells = new CellWrapper.Builder()
//                .add("cell1",
//                        colorBuilder.initialize(expansionHubDevices.getI2C(0)).setFields(0.152 * tresh, 0.184 * treshG, 0.149 * tresh, 0.0).get(),
//                        colorBuilder.initialize(expansionHubDevices.getI2C(1)).setFields(0.192 * tresh, 0.203 * treshG, 0.170 * tresh, 0.0).get())
//                .add("cell2",
//                        colorBuilder.initialize(expansionHubDevices.getI2C(2)).setFields(0.146 * tresh, 0.187 * treshG, 0.148 * tresh, 0.0).get(),
//                        colorBuilder.initialize(expansionHubDevices.getI2C(3)).setFields(0.115 * tresh, 0.120 * treshG, 0.104 * tresh, 0.0).get())
//                .add("cell3",
//                        colorBuilder.initialize(controlHubDevices.getI2C(0)).setFields(0.153 * tresh, 0.175 * treshG, 0.149 * tresh, 0.0).get(),
//                        colorBuilder.initialize(controlHubDevices.getI2C(1)).setFields(0.136 * tresh, 0.145 * treshG, 0.120 * tresh, 0.0).get());

        cells = new CellWrapper.Builder();

//        cells.getCell("cell1").tresholder = new double[]{0.123 * tresh,0.154 * treshG,0.122 * tresh};
//        cells.getCell("cell2").tresholder = new double[]{0.133 * tresh,0.155 * treshG,0.114 * tresh};
//        cells.getCell("cell3").tresholder = new double[]{0.123 * tresh,0.154 * treshG,0.122 * tresh};

        servosCollector
                .add(servoBuilder.initialize(pusher0).setFields(1000.0, 180.0).get())
                .add(servoBuilder.initialize(pusher2).setFields(750.0, 180.0).get())
                .add(servoBuilder.initialize(pusher1).setFields(1000.0, 180.0).get());

        prepareServo(0);
        prepareServo(1);
        prepareServo(2);
        sayCreated();
    }
    /*1 - в массиве это зелёный шар
     * 2 - в массиве это фиолетовый*/
    private int artifactCount;
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

        cells.showAll();
        servosCollector.showData();
    }
    public void fire(int color){
        if(!isInitialized) return;
        triggeredCell = cells.getNeededCell(color);

        if (triggeredCell == null) {
            return;
        }
        switch (triggeredCell.name){
            case "cell1":
                servosCollector.get(pusher0).execute(0.47);
                triggeredServo = servosCollector.get(pusher0);
                break;
            case "cell2":
                servosCollector.get(pusher2).execute(0.8);
                triggeredServo = servosCollector.get(pusher2);
                break;
            case "cell3":
                servosCollector.get(pusher1).execute(0.48);
                triggeredServo = servosCollector.get(pusher1);
                break;
        }
    }
    public void fireCell(int cellNum){
        switch (cellNum){
            case 0:
                triggeredCell = cells.getCell("cell3");
                servosCollector.get(pusher1).execute(0.5);
                triggeredServo = servosCollector.get(pusher1);
                break;
            case 1:
                triggeredCell = cells.getCell("cell2");
                servosCollector.get(pusher2).execute(0.73);
                triggeredServo = servosCollector.get(pusher2);
                break;
            case 2:
                triggeredCell = cells.getCell("cell1");
                servosCollector.get(pusher0).execute(0.58);
                triggeredServo = servosCollector.get(pusher0);
                break;
        }
    }
    public void prepareServo(int cellNum){
        if(!isInitialized) return;
        switch (cellNum){
            case 0:
                triggeredCell = cells.getCell("cell3");
                servosCollector.get(pusher1).execute(0.06);
                triggeredServo = servosCollector.get(pusher1);
                break;
            case 1:
                triggeredCell = cells.getCell("cell2");
                servosCollector.get(pusher2).execute(0.05);
                triggeredServo = servosCollector.get(pusher2);
                break;
            case 2:
                triggeredCell = cells.getCell("cell1");
                servosCollector.get(pusher0).execute(0.05);
                triggeredServo = servosCollector.get(pusher0);
                break;
        }
    }

    public void prefirePos(int cellNum){
        switch (cellNum){
            case 0:
                triggeredCell = cells.getCell("cell3");
                servosCollector.get(pusher1).execute(0.19);
                triggeredServo = servosCollector.get(pusher1);
                break;
            case 1:
                triggeredCell = cells.getCell("cell2");
                servosCollector.get(pusher2).execute(0.27);
                triggeredServo = servosCollector.get(pusher2);
                break;
            case 2:
                triggeredCell = cells.getCell("cell1");
                servosCollector.get(pusher0).execute(0.19);
                triggeredServo = servosCollector.get(pusher0);
                break;
        }
    }

    public boolean isAllReady(){
        cells.getCell("cell1").bcNeededColor = false;
        cells.getCell("cell1").bcSomeColor = false;
        cells.getCell("cell2").bcNeededColor = false;
        cells.getCell("cell2").bcSomeColor = false;
        cells.getCell("cell3").bcNeededColor = false;
        cells.getCell("cell3").bcSomeColor = false;
        return !servosCollector.get(pusher0).isBusy(8) && !servosCollector.get(pusher1).isBusy(8) && !servosCollector.get(pusher2).isBusy(8);
    }
    public static class CellWrapper extends UpdatableModule {
        public ArrayList<ColorSensorWrapper> sensorsWrapper = new ArrayList<>();
        public boolean isInit;
        public String name;
        public boolean bcSomeColor = false;
        public boolean bcNeededColor = false;
        public CellWrapper(String cellName, ColorSensorWrapper...sensorsWrapperIn){
            super(cellName);
            this.name = cellName;

            sensorsWrapper.addAll(Arrays.asList(sensorsWrapperIn));
        }
        public double[] tresholder = new double[3];
        public double r, g, b;
        public int isFounded(){
            int count = 0;
            if(foundedColor != 0) count |= 1;
            return count;
        }
        public int getColor(){
            int clr = 0;
            for (ColorSensorWrapper color : sensorsWrapper) {
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

        public double foundedColor;
        @Override
        protected void updateExt() {
            boolean redDetected = false;
            boolean greenDetected = false;
            boolean blueDetected = false;
            boolean nothingDetected = false;

            for (ColorSensorWrapper color:sensorsWrapper) {
                color.update();
                if(color.rgba != null){
                    r += color.rgba.red;
                    g += color.rgba.green;
                    b += color.rgba.blue;
                }
                else {
                    r = 0;
                    g = 0;
                    b = 0;
                }
            }
            r /= 2;
            g /= 2;
            b /= 2;

            double tRU = 1.12;
            double tRD = 0.96;
            double tGD = 0.8;
            double tGU = 1;
            double tBD = 0.85;
            double tBU = 1;

            if(name.equals("cell1")){
                //1 зеленный полностью накрывает 231 416 308 в темноте 123 154 122
                //1 фиол полностью накрывает 306 325 344
                if(r > g && r > b && (r > 0.231 * tRD && r < 0.306 * tRU)) {
                    redDetected = true;}
                else if(g > r && g > b && (g > 0.325 * tGD && g < 0.416 * tGU)) {
                    greenDetected = true;}
                else if(b > r && b > g && (b > 0.308 * tBD && b < 0.344 * tBU )) {
                    blueDetected = true;
                }else nothingDetected = true;
                if (!nothingDetected){
                    foundedColor = redDetected || blueDetected ? 2 : 1;
                }else foundedColor = 0;
            } else if (name.equals("cell2")) {
                //2 зелённый  243 397 289                              133 155 114
                //2 purple 284 280 292
                if(r > g && r > b && (r > 0.243 * tRD && r < 0.284 * tRU)) {
                    redDetected = true;}
                else if(g > r && g > b && (g > 0.280 * tGD && g < 0.397 * tGU)) {
                    greenDetected = true;}
                else if(b > r && b > g && (b > 0.289 * tBD && b < 0.292 * tBU)) {
                    blueDetected = true;
                }else nothingDetected = true;
                if (!nothingDetected){
                    foundedColor = redDetected || blueDetected ? 2 : 1;
                }else foundedColor = 0;
            } else if (name.equals("cell3")) {
                //3 green 180 318 252                                  123 154 122
                //3purple 265 271 312
                if(r > g && r > b && (r > 0.180 * tRD && r < 0.265 * tRU)) {
                    redDetected = true;}
                else if(g > r && g > b && (g > 0.271 * tGD && g < 0.318 * tGU)) {
                    greenDetected = true;}
                else if(b > r && b > g && (b > 0.252 * tBD && b < 0.312 * tBU)) {
                    blueDetected = true;
                }else nothingDetected = true;
                if (!nothingDetected){
                    foundedColor = redDetected || blueDetected ? 2 : 1;
                }else foundedColor = 0;
            }

        }

        @Override
        protected void showDataExt() {
            telemetry.addData("Reasons", "byColor %s byNotEmpty %s", bcNeededColor, bcSomeColor);
            telemetry.addData("color", foundedColor);
            telemetry.addData("Data", "r %.3f g %.3f b %.3f", r, g, b);
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
                    if (cell.foundedColor == color) {
                        cell.bcSomeColor = false;
                        cell.bcNeededColor = true;
                        found = cell;
                        break;}
                    else {
                        if(cell.foundedColor != 0) {
                            cell.bcNeededColor = false;
                            cell.bcSomeColor = true;
                            found = cell;
                        }
                    }
                }
                return found;
            }
            public CellWrapper getFullCell(){
                CellWrapper found = null;
                for (CellWrapper cell : cells.values()) {
                    if (cell.foundedColor != 0) {
                        cell.bcSomeColor = true;
                        found = cell;
                        break;}
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
