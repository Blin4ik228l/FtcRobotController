package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private int numberOfLoadedArtifacts = 0;
    private final Cells cells;
    private final ElapsedTime runtime;
    private final ElapsedTime runtime2;
    public boolean isRotateBaraban;
    public boolean isTurnOn;
    public boolean isFlyWheelOn;
    public double barabanPos;
    public boolean isArtifactsLoading = true;
    public boolean isArtifactFiring = false;
    public void setAll(boolean isTurnOn, boolean isFlyWheelOn, double velocity){
        collector.motors.turnOnInTake(isTurnOn);
//        collector.motors.turnOnFlyWheel(isFlyWheelOn);
//        collector.encoders.setVelocities(velocity);

        update();
        calculate();
        action();
    }
    public void update(){
        collector.colorSensor.update();

        loadedArtifactColor = collector.colorSensor.currentArtifact;
    }
    public void calculate(){
        if(loadedArtifactColor != 0 && numberOfLoadedArtifacts != 3 && runtime.seconds() > 1) {

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
            }

            if(barabanPos != 1)barabanPos += 0.5;

            runtime.reset();
        }

//        if(isRobotFiring){
//            if(loadedArtifacts.get(count2) != null && count2 != -1){
//                isFounded = loadedArtifacts.get(count2).color == driveTrain.exOdometry.camera.randomizedArtifact[count] ? true : false;
//
//                if(!isFounded){
//                    count2--;
//                }else {
//                    barabanTargetPos = loadedArtifacts.get(count2).pos;
//
//                    if(isCurCellIsEmpty() && runtime2.seconds() > 1){
//                        loadedArtifacts.remove(count2);
//                        count2 = loadedArtifacts.size() - 1;
//                        count++;
//                        runtime2.reset();
//                    }
//
//                }
//            }
//        }
    }

    public void action(){
        if(isArtifactsLoading){
            collector.encoders.setVelocities(0);
            collector.servos.getBaraban().setPosition(barabanPos);
        }else {
            //Добавить задержку
            collector.encoders.setVelocities(300);
            collector.servos.getPusher().setPosition(0.3);
        }

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
}
