package org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class DigitalCellsClass extends Module {
    private final ServomotorsClass servomotorsClass;
    private final Cell cell0, cell1, cell2;

    public DigitalCellsClass(ServomotorsClass servomotorsClass, OpMode op){
        super(op.telemetry);
        this.servomotorsClass = servomotorsClass;

        cell0 = new Cell(0, new Cell.Table(0, BARABAN_CELL0_POS));
        cell1 = new Cell(1, new Cell.Table(0, BARABAN_CELL1_POS));
        cell2 = new Cell(2, new Cell.Table(0, BARABAN_CELL2_POS));

        randomizedArtifact = new int[3];
    }
    /*0 - в массиве это зелёный шар
     * 1 - в массиве это фиолетовый*/
    private int[] randomizedArtifact;
    private int artifactCount;
    private int targCell;
    private int curCell;
    public boolean isMaxed;

    public int getArtifactCount() {
        return artifactCount;
    }

    public void checkNumberOfArtifacts(){
        int artifactNumber = 0;

        artifactNumber += cell0.table.color != 0 ? 1 : 0;
        artifactNumber += cell1.table.color != 0 ? 1 : 0;
        artifactNumber += cell2.table.color != 0 ? 1 : 0;

        artifactCount = artifactNumber;
    }

    public void setColor(int color){
        if(servomotorsClass.curBarabanPos == BARABAN_CELL0_POS){
            cell0.table.color = color;
        } else if (servomotorsClass.curBarabanPos == BARABAN_CELL1_POS) {
            cell1.table.color = color;
        } else if (servomotorsClass.curBarabanPos == BARABAN_CELL2_POS) {
            cell2.table.color = color;
        }else telemetry.addLine("there is nothing");

        checkNumberOfArtifacts();

        if (artifactCount == 3) isMaxed = true;
    }

    public int getEmptyCell() {
        if(servomotorsClass.curBarabanPos == BARABAN_CELL2_POS)
        {
            curCell = 2;
            if(cell2.table.color == 0)
            {
                targCell = 2;
            }
            else if (cell1.table.color == 0)
            {
                targCell = 1;
            }
            else
            {
                targCell = 0;
            }
        }
        else if (servomotorsClass.curBarabanPos == BARABAN_CELL1_POS)
        {
            curCell = 1;
            if(cell1.table.color == 0)
            {
                targCell = 1;
            }
            else if (cell0.table.color == 0)
            {
                targCell = 0;
            }
            else
            {
                targCell = 2;
            }
        }
        else
        {
            curCell = 0;
            if(cell0.table.color == 0)
            {
                targCell = 0;
            }
            else if (cell1.table.color == 0)
            {
                targCell = 1;
            }
            else
            {
                targCell = 2;
            }
        }

        return targCell;
    }
    public int getFullCell(){
        if(servomotorsClass.curBarabanPos == BARABAN_CELL2_POS)
        {
            curCell = 2;
            if(cell2.table.color != 0)
            {
                targCell = 2;
            }
            else if (cell1.table.color != 0)
            {
                targCell = 1;
            }
            else
            {
                targCell = 0;
            }
        }
        else if (servomotorsClass.curBarabanPos == BARABAN_CELL1_POS)
        {
            curCell = 1;
            if(cell1.table.color != 0)
            {
                targCell = 1;
            }
            else if (cell0.table.color != 0)
            {
                targCell = 0;
            }
            else
            {
                targCell = 2;
            }
        }
        else
        {
            curCell = 0;
            if(cell0.table.color != 0)
            {
                targCell = 0;
            }
            else if (cell1.table.color != 0)
            {
                targCell = 1;
            }
            else
            {
                targCell = 2;
            }
        }

        return targCell;
    }
    public int findNeededCell(int color){
        if(cell2.table.color == color){
            return 2;
        }
        if(cell1.table.color == color){
            return 1;
        }

        if(cell0.table.color == color){
            return 0;
        }

        if(cell2.table.color != 0) return 2;
        else if(cell1.table.color != 0) return 1;
        else return 0;
    }
    public void deleteColorFromCell(){
        findNeededCell().table.color = 0;
        checkNumberOfArtifacts();
        if (artifactCount == 0) isMaxed = false;
    }
    public Cell findNeededCell(){
        return cell0.table.pos == servomotorsClass.curBarabanPos ? cell0 : (cell1.table.pos == servomotorsClass.curBarabanPos ? cell1 : cell2);
    }
    public static class Cell{
        public Cell(int numCell, Table table){
            this.numCell = numCell;
            this.table = table;
        }
        int numCell;
        public Table table;
        public static class Table{
            public Table(int color, double pos){
                this.color = color;
                this.pos = pos;
            }
            double color;
            double pos;
        }
    }

    public String getColorFromNumber(double number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    public String getColorFromNumber(int number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }
    public void setRandomizedArtifact(int[] randomizedArtifact) {
        this.randomizedArtifact = randomizedArtifact;
    }
    public int[] getRandomizedArtifact() {
        return randomizedArtifact;
    }
    @Override
    public void showData() {
        telemetry.addLine("===DIGITAL CELLS===");
        telemetry.addData("Count", artifactCount);
        telemetry.addData("Randomized artifacts:", "%s %s %s", getColorFromNumber(randomizedArtifact[0]), getColorFromNumber(randomizedArtifact[1]), getColorFromNumber(randomizedArtifact[2]));
        telemetry.addData("Cur cell", curCell);
        telemetry.addData("Cells", "C0:%s C1:%s C2:%s", getColorFromNumber(cell0.table.color),getColorFromNumber(cell1.table.color),getColorFromNumber(cell2.table.color));
        telemetry.addLine();
    }
}
