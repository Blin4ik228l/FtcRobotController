package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts.Servos;

public class AutomaticClass extends Module {
    public AutomaticClass(OpMode op, RobotClass.Collector collector) {
        super(op.telemetry);

        cells = new Cells();
        runtime = new ElapsedTime();
        runtime2 = new ElapsedTime();

        this.collector = collector;
    }
    public RobotClass.Collector collector;
    private int loadedArtifactColor;
    private double curDistance;
    private int numberOfLoadedArtifacts = 0;
    private int[] randomizedArtifacts;
    private int count;
    private final Cells cells;
    private final ElapsedTime runtime;
    private final ElapsedTime runtime2;
    public boolean isRotateBaraban;
    public boolean isTurnOn;
    public boolean isFlyWheelOn;
    public double barabanPos;
    public double lastPos;
    public boolean isArtifactsLoading = true;
    public boolean isArtifactFiring = false;
    public boolean isNowFiring;
    public boolean isStopFiring = false;
    public boolean isWasCount = false;

    public void setAll(boolean isTurnOn, boolean isFlyWheelOn, double velocity){
        if(isTurnOn){
            update();
            calculate();
            action();
        }
    }
    public void update(){
        collector.colorSensor.update();
        collector.encoders.updateAll();

        loadedArtifactColor = collector.colorSensor.currentArtifact;
        curDistance = collector.colorSensor.curDistance;
    }
    public void calculate(){
        if(loadedArtifactColor != 0 && numberOfLoadedArtifacts != 3 && runtime.seconds() > 1 && !isArtifactFiring && isArtifactsLoading) {
            numberOfLoadedArtifacts++;

            if(numberOfLoadedArtifacts == 1){
                cells.cell0.table.color = loadedArtifactColor;
                cells.cell0.table.pos = collector.servos.getBaraban().getPosition();

            }
            if(numberOfLoadedArtifacts == 2){
                cells.cell1.table.color = loadedArtifactColor;
                cells.cell1.table.pos = collector.servos.getBaraban().getPosition();
            }
            if(numberOfLoadedArtifacts == 3){
                cells.cell2.table.color = loadedArtifactColor;
                cells.cell2.table.pos = collector.servos.getBaraban().getPosition();

                isArtifactsLoading = false;
                isArtifactFiring = true;
            }

            if(barabanPos != 1)barabanPos += 0.5;

            runtime.reset();
        }

        if(runtime2.seconds() > 1 && count != 3 && !isArtifactsLoading && isArtifactFiring){

            lastPos = barabanPos;

            if(randomizedArtifacts[0] == 0 && randomizedArtifacts[1] == 0 && randomizedArtifacts[2] == 0) {
                return;}

            if(cells.cell0.table.color == randomizedArtifacts[count]){
                barabanPos = cells.cell0.table.pos;
            }
            if(cells.cell1.table.color == randomizedArtifacts[count]){
                barabanPos = cells.cell1.table.pos;
            }
            if(cells.cell2.table.color == randomizedArtifacts[count]){
                barabanPos = cells.cell2.table.pos;
            }
            runtime2.reset();
        }
    }

    public void deleteArtifact(){
        if(loadedArtifactColor == 0 && isArtifactFiring){
            cells.cell0.table.color = cells.cell0.table.pos == barabanPos ? 0 : cells.cell0.table.color;
            cells.cell1.table.color = cells.cell1.table.pos == barabanPos ? 0 : cells.cell1.table.color;
            cells.cell2.table.color = cells.cell2.table.pos == barabanPos ? 0 : cells.cell2.table.color;

            if(count != 3)count++;
            else {
                isArtifactsLoading = false;
                isArtifactFiring = false;
            }
        }
    }

    public void action(){
        if(isArtifactsLoading){
            collector.motors.turnOnInTake(true);
            collector.encoders.setVelocities(-1);
            collector.servos.getBaraban().setPosition(barabanPos);
        }

        if(isArtifactFiring) {
            //Добавить задержку
            collector.motors.turnOnInTake(false);
            collector.servos.getBaraban().setPosition(barabanPos);
            collector.encoders.setVelocities(5);


            if (runtime.seconds() > 2 && loadedArtifactColor != 0 && curDistance < 10) {
                collector.servos.getPusher().setPosition(0.7);
            }

            if(runtime.seconds() > 4 && loadedArtifactColor == 0 && curDistance >= 10){
                deleteArtifact();
                collector.servos.getPusher().setPosition(0.45);
                runtime.reset();
            }
        }

        if(!isArtifactFiring && !isArtifactsLoading){
            collector.servos.getPusher().setPosition(PUSHER_START_POS);
            collector.servos.getBaraban().setPosition(0);
            collector.encoders.setVelocities(0);

            cells.cell0.table.pos = collector.servos.getBaraban().getPosition();
            cells.cell1.table.pos = collector.servos.getBaraban().getPosition();
            cells.cell2.table.pos = collector.servos.getBaraban().getPosition();

            count = 0;
            numberOfLoadedArtifacts = 0;
            isArtifactFiring = false;
            isArtifactsLoading = true;

            runtime.reset();
            runtime2.reset();
        }
    }

    public void setRandomizedArtifacts(int[] artifacts){
        randomizedArtifacts = artifacts;
    }
    public static class Cells {
        public Cells(){
            cell0 = new Cell(0, new Cell.Table(0, 0.0));
            cell1 = new Cell(1, new Cell.Table(0, 0.0));
            cell2 = new Cell(2, new Cell.Table(0, 0.0));
        }
        private final Cell cell0, cell1, cell2;
        public static class Cell{
            public Cell(int numCell, Table table){
                this.numCell = numCell;
                this.table = table;
            }
            int numCell;
            Table table;

            public static class Table{
                public Table(int color, double pos){
                    this.color = color;
                    this.pos = pos;
                }
                int color;
                double pos;
            }
        }
    }
    public void showCells(){
        telemetry.addData("cell0", cells.cell0.table.color + " " + cells.cell0.table.pos);
        telemetry.addData("cell1", cells.cell1.table.color + " " + cells.cell1.table.pos);
        telemetry.addData("cell2", cells.cell2.table.color + " " + cells.cell2.table.pos);
    }

    public void showCount(){
        telemetry.addData("count", count);
        telemetry.addData("runtime2.seconds() >", runtime2.seconds());
        telemetry.addData("isArtifactsLoading", isArtifactsLoading);
        telemetry.addData("runtime.seconds()", runtime.seconds());
        telemetry.addData("numberOfLoadedArtifacts", numberOfLoadedArtifacts);
        telemetry.addData("curDistance", curDistance);
    }
}
