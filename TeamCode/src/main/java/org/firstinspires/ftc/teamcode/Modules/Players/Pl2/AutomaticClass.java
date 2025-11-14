package org.firstinspires.ftc.teamcode.Modules.Players.Pl2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Players.PlayerClass;
import org.firstinspires.ftc.teamcode.Robot.MotorsControllerClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class AutomaticClass extends PlayerClass {
    public AutomaticClass(Gamepad gamepad, RobotClass.Collector collector, OpMode op) {
        super(new JoystickActivity(gamepad, op.telemetry), op.telemetry);
        this.collector = collector;

        cells = new Cells();
        runtime = new ElapsedTime();

        motorsController = new MotorsControllerClass(collector.motors, telemetry);
    }
    public final MotorsControllerClass motorsController;
    public RobotClass.Collector collector;
    private double loadedArtifactColor;
    private double loadedArtifactColor2;
    private double curDistance;
    private double curDistance2;
    private final Cells cells;
    private final ElapsedTime runtime;
    public boolean isWhileFiring = false;

    public int[] randomizedArtifacts = new int[3];
    public boolean isVyrCompleted;
    public double range;
    public double curVel;
    public int artifactCount;

    public void setFields(boolean isVyrCompleted, double range, double curVel){
        this.isVyrCompleted = isVyrCompleted;
        this.range = range;
        this.curVel = curVel;
    }
    public void setRandomizedArtifacts(int[] randomizedArtifacts){
        this.randomizedArtifacts = randomizedArtifacts;
    }

    public void execute(){
        if(!joystickActivity.buttonX) {
            setFieldsInMotor(0,0,0);
        }
        checkNumberOfArtifacts();

        if(!isWhileFiring) {
            if (artifactCount == 0) {
                actionLOAD(0);
            }else if (artifactCount == 1) {
                actionLOAD(1);
            }else if (artifactCount == 2) {
                actionLOAD(2);
            }else {
                setFieldsInMotor(1, 0, 1.5);
                setFieldsInMotor(0, 0, 0);
                isWhileFiring = true;
            }
        }
        else{
            if(isRandomizeWasDetected() && joystickActivity.buttonY ){
                if(artifactCount == 3 ){
                    actionFIRE(0);
                }else if(artifactCount == 2){
                    actionFIRE(1);
                }else if(artifactCount == 1 ){
                    actionFIRE(2);
                }else {
                    setFieldsInMotor(0,0,0.5);
                    push(PUSHER_START_POS, 0.3);
                    next(BARABAN_START_POS, 0.8);
                    isWhileFiring = false;
                }
            }
        }
    }
    public void setFieldsFromColorSensor(){
        loadedArtifactColor = collector.colorSensor.currentArtifact;
        loadedArtifactColor2 = collector.colorSensor.currentArtifact1;

        curDistance = collector.colorSensor.curDistance;
        curDistance2 = collector.colorSensor.curDistance1;
    }
    public void loadArtifactInCell(int cell){
        if(cell == 0){
            cells.cell0.table.color = loadedArtifactColor;
            if (cells.cell0.table.color == 0) cells.cell0.table.color = loadedArtifactColor2;
            cells.cell0.table.pos = 0.0;
        } else if (cell == 1) {
            cells.cell1.table.color = loadedArtifactColor;
            if (cells.cell1.table.color == 0) cells.cell1.table.color = loadedArtifactColor2;
            cells.cell1.table.pos = 0.5;
        } else if (cell == 2) {
            cells.cell2.table.color = loadedArtifactColor;
            if (cells.cell2.table.color == 0) cells.cell2.table.color = loadedArtifactColor2;
            cells.cell2.table.pos = 1.0;
        }else {
            return;
        }
    }
    public void deleteColorFromCell(){
        findNeededCell().table.color = 0;
    }
    public double findNeededArtifactPos(int color){
        if(collector.servos.getBaraban().getPosition() == cells.cell0.table.pos) return cells.cell0.table.color == color ? cells.cell0.table.pos  : (cells.cell1.table.color == color ? cells.cell1.table.pos : (cells.cell2.table.color == color ?  cells.cell2.table.pos: collector.servos.getBaraban().getPosition()));
        else if (collector.servos.getBaraban().getPosition() == cells.cell1.table.pos ) return cells.cell1.table.color == color ? cells.cell1.table.pos  : (cells.cell0.table.color == color ? cells.cell0.table.pos : (cells.cell2.table.color == color ?  cells.cell2.table.pos: collector.servos.getBaraban().getPosition()));
        else return cells.cell2.table.color == color ? cells.cell2.table.pos  : (cells.cell1.table.color == color ? cells.cell1.table.pos : (cells.cell0.table.color == color ?  cells.cell0.table.pos: collector.servos.getBaraban().getPosition()));
    }

    public void checkNumberOfArtifacts(){
        int artifactNumber = 0;

        artifactNumber += cells.cell0.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell1.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell2.table.color != 0 ? 1 : 0;

        artifactCount = artifactNumber;
    }
    public boolean isArtifactInIt(){
        setFieldsFromColorSensor();
        return (loadedArtifactColor != 0 || loadedArtifactColor2 != 0) && (curDistance < 9);
    }
    public Cells.Cell findNeededCell(){
        return cells.cell0.table.pos == collector.servos.getBaraban().getPosition() ? cells.cell0 : (cells.cell1.table.pos == collector.servos.getBaraban().getPosition() ? cells.cell1 : cells.cell2);
    }
    public double findNeededPosAngle(double length){
        double neededPos = 0;

//        if(getHeight(getAngle(length)) >= 94){
//            return getAngle(length)/Math.toRadians(30);
//        }

        return collector.servos.getAngle().getPosition();
    }
    double getAngle(double length){
        double velBall = 450;

        return (length * 981) / (Math.pow(velBall, 2)) / 2.0;
    }
    double getHeight(double angle){
        double velBall = 450;

        return (Math.pow(velBall, 2) * Math.pow(Math.sin(angle), 2)) / (981 * 2);
    }

    public boolean isRobotHaveMinVel(){
        return curVel < 25;
    }
    public boolean isAllowFire(){
        return isVyrCompleted;
    }
    public boolean isRandomizeWasDetected(){
        return randomizedArtifacts[0] != 0;
    }
    public void actionFIRE(int num){
        angle(findNeededPosAngle(range), 0);
        next(findNeededArtifactPos(randomizedArtifacts[num]), 0.8);

        if(isRobotHaveMinVel() && joystickActivity.buttonY) {
            waitWhile(4.5);
            push(0.9, 0.6);
            push(0.45, 0.4);
        }else{return;}

        if(!isArtifactInIt()){
            deleteColorFromCell();
        }
    }
    public void actionLOAD(int num){
        setFieldsInMotor(-1, -1, 0);

        if (isArtifactInIt()) {
            loadArtifactInCell(num);

            if(num == 0) next(0.5, 0.8);
            else if (num == 1) next(1, 0.8);
        }
    }
    public void next(double pos, double time){
        if(collector.servos.getBaraban().getPosition() == pos && !joystickActivity.buttonX) return;

        runtime.reset();
        while (runtime.seconds() < time && joystickActivity.buttonX) collector.servos.getBaraban().setPosition(pos);
    }
    public void push(double pos, double time){
        if(collector.servos.getPusher().getPosition() == pos && !joystickActivity.buttonX) return;

        runtime.reset();
        while (runtime.seconds() < time && joystickActivity.buttonX) collector.servos.getPusher().setPosition(pos);
    }
    public void angle(double pos, double time){
        if(collector.servos.getAngle().getPosition() == pos && !joystickActivity.buttonX) return;

        runtime.reset();
        while (runtime.seconds() < time && joystickActivity.buttonX) collector.servos.getAngle().setPosition(pos);
    }
    public void sleep(double seconds){
        runtime.reset();
        while (true){
            if (!(runtime.seconds() < seconds)) break;
        }
    }
    public void setFieldsInMotor(double inTake, double radianSpeed, double time){
        if(motorsController.radianSpeed == radianSpeed && motorsController.inTakePower == inTake && time == 0 && !joystickActivity.buttonX) return;

        motorsController.inTakePower = inTake;
        motorsController.radianSpeed = radianSpeed;

        runtime.reset();
        while (joystickActivity.buttonX){
            if (!(runtime.seconds() < time)) break;
        }
    }
    public void waitWhile(double speed){
        setFieldsInMotor(0, speed, 0);

        while(joystickActivity.buttonX){
            if(Math.abs(collector.encoders.getVelocity()) >= speed) break;
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
                double color;
                double pos;
            }
        }
    }
    public void showData(){
        telemetry.addData("Y", joystickActivity.buttonY);
        telemetry.addData("X", joystickActivity.buttonX);
        telemetry.addLine("Automatic class caption")
                .addData("Time in seconds", "%.1f %n", runtime.seconds())
                .addData("Artifacts number", artifactCount)
                .addData("cell0", "pos[%s] color[%s]%n", cells.cell0.table.pos, getColorFromNumber(cells.cell0.table.color))
                .addData("cell1", "pos[%s] color[%s]%n", cells.cell1.table.pos, getColorFromNumber(cells.cell1.table.color))
                .addData("cell2", "pos[%s] color[%s]%n", cells.cell2.table.pos, getColorFromNumber(cells.cell2.table.color));
        telemetry.addLine();
        collector.colorSensor.showData();
        collector.servos.showData();
        collector.encoders.showData();
    }
    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
}
