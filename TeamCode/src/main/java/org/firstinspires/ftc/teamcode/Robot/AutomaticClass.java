package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.Player1;
import org.firstinspires.ftc.teamcode.Modules.Players.Player;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.MotorsOnCollector;

public class AutomaticClass extends Module implements Runnable{
    public AutomaticClass(RobotClass.Collector collector, Player.JoystickActivity joystickActivity1, OpMode op) {
        super(op.telemetry);

        cells = new Cells();
        runtime = new ElapsedTime();

        this.collector = collector;
        joystickActivityPl1 = joystickActivity1;

        motorsController = new MotorsController(collector.motors);
        motorsController.start();
    }
    public Player.JoystickActivity joystickActivityPl1;
    public RobotClass.MecanumDrivetrain drivetrain;
    public RobotClass.Collector collector;
    private double loadedArtifactColor;
    private double loadedArtifactColor2;
    private double curDistance;
    private double curDistance2;
    private final Cells cells;
    private final ElapsedTime runtime;
    public boolean isWhileFiring = false;
    public int[] randomizedArtifact = new int[3];
    public boolean isVyrCompleted;
    public double range;

    private final MotorsController motorsController;
    public static class MotorsController extends Thread{
        private final MotorsOnCollector motors;
        public double inTakePower;
        public double radianSpeed;
        public MotorsController(MotorsOnCollector motorsOnCollector){
            motors = motorsOnCollector;
        }
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()){
                execute();
            }
        }
        public void execute(){
            motors.turnOnInTake(inTakePower);
            motors.setSpeedOnFlyWheel(radianSpeed);
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()){
            execute();
        }
    }
    public void execute(){
        if(checkNumberOfArtifacts() != 3 && !isWhileFiring ) {
            if (checkNumberOfArtifacts() == 0) {
                actionLOAD(0);
            }else if (checkNumberOfArtifacts() == 1) {
                actionLOAD(1);
            }else if (checkNumberOfArtifacts() == 2) {
                actionLOAD(2);
            }else{
                setFieldsInMotor(1, 0, 1.5);
                setFieldsInMotor(0, 0, 0);
                isWhileFiring = true;
            }
        }else{
            if(isRandomizeWasDetected() && isButtonY()){
                if(checkNumberOfArtifacts() == 3){
                    actionFIRE(0);
                }else if(checkNumberOfArtifacts() == 2){
                    actionFIRE(1);
                }else if(checkNumberOfArtifacts() == 1){
                    actionFIRE(2);
                }else {
                    if(checkNumberOfArtifacts() == 0){
                        setFieldsInMotor(0,0,0);
                        push(PUSHER_START_POS);
                        next(BARABAN_START_POS);
                        isWhileFiring = false;
                    }
                }
            }
        }
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
    public void update(){
        collector.colorSensor.update();

        loadedArtifactColor = collector.colorSensor.currentArtifact;
        loadedArtifactColor2 = collector.colorSensor.currentArtifact1;

        curDistance = collector.colorSensor.curDistance;
        curDistance2 = collector.colorSensor.curDistance1;
    }
    public void next(double pos){
        if(collector.servos.getBaraban().getPosition() == pos) return;

        runtime.reset();
        while (runtime.seconds() < 0.3) collector.servos.getBaraban().setPosition(pos);
    }
    public void push(double pos){
        if(collector.servos.getPusher().getPosition() == pos) return;

        runtime.reset();
        while (runtime.seconds() < 0.4) collector.servos.getPusher().setPosition(pos);
    }
    public void setAngle(double pos){
        if(collector.servos.getAngle().getPosition() == pos) return;

        runtime.reset();
        while (runtime.seconds() < 0.2) collector.servos.getAngle().setPosition(pos);
    }
    public void sleep(double seconds){
        runtime.reset();
        while (true){
            if (!(runtime.seconds() < seconds)) break;
        }
    }
    public void setFieldsInMotor(double inTake, double radianSpeed, double time){
        if(motorsController.radianSpeed == radianSpeed && motorsController.inTakePower == inTake) return;

        motorsController.inTakePower = inTake;
        motorsController.radianSpeed = radianSpeed;

        runtime.reset();
        while (true){
            if (!(runtime.seconds() < time)) break;
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

    public int checkNumberOfArtifacts(){
        int artifactNumber = 0;

        artifactNumber += cells.cell0.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell1.table.color != 0 ? 1 : 0;
        artifactNumber += cells.cell2.table.color != 0 ? 1 : 0;

        return artifactNumber;
    }
    public boolean isArtifactInIt(){
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
        return drivetrain.exOdometry.robotSelfCentricVel.length() < 25;
    }
    public boolean isButtonY(){
        return joystickActivityPl1.buttonY;
    }
    public boolean isAllowFire(){
        return isVyrCompleted;
    }
    public boolean isRandomizeWasDetected(){
        return randomizedArtifact[0] != 0;
    }
    public boolean isButtonX(){
        return joystickActivityPl1.buttonX;
    }
    public void actionFIRE(int num){
        if(!isButtonX()) {
            setFieldsInMotor(0, 0, 0);
            return;}

        setFieldsInMotor(0, 5,0);
        push(0.45);
        update();
        if(!isArtifactInIt()){
            deleteColorFromCell();
            sleep(0.3);
        }else{
            next(findNeededArtifactPos(randomizedArtifact[num]));

            setAngle(findNeededPosAngle(range));
            if(isRobotHaveMinVel() && isButtonY() && isAllowFire()) {
                push(0.9);}
        }
    }
    public void actionLOAD(int num){
        if(!isButtonX()) {
            setFieldsInMotor(0, 0, 0);
            return;}

        setFieldsInMotor(-1, -1, 0);
        update();
        if (isArtifactInIt()) {
            loadArtifactInCell(num);
            sleep(0.3);
            next(0.5);
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
        telemetry.addLine("Automatic class caption")
                .addData("Time in seconds", "%.1f %n", runtime.seconds())
                .addData("Artifacts number", checkNumberOfArtifacts())
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
