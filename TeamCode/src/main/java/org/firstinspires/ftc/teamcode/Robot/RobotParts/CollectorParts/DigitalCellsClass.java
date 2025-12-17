package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class DigitalCellsClass extends Module {

    public DigitalCellsClass(ServomotorsClass servos, OpMode op){
        super(op.telemetry);
        this.servos = servos;
        cell0 = new Cell(0, new Cell.Table(0, BARABAN_CELL0_POS));
        cell1 = new Cell(1, new Cell.Table(0, BARABAN_CELL1_POS));
        cell2 = new Cell(2, new Cell.Table(0, BARABAN_CELL2_POS));
    }
    private int[] randomizedArtifact = new int[3];
    /*0 - в массиве это зелёный шар
     * 1 - в массиве это фиолетовый*/
    private final Cell cell0, cell1, cell2;
    private final ServomotorsClass servos;
    public int artifactCount;
    private int targCell;
    private int curCell;
    public void checkNumberOfArtifacts(){
        int artifactNumber = 0;

        artifactNumber += cell0.table.color != 0 ? 1 : 0;
        artifactNumber += cell1.table.color != 0 ? 1 : 0;
        artifactNumber += cell2.table.color != 0 ? 1 : 0;

        artifactCount = artifactNumber;
    }

    public void setColor(int color){
        if(servos.curBarabanPos == BARABAN_CELL0_POS){
            cell0.table.color = color;
        } else if (servos.curBarabanPos == BARABAN_CELL1_POS) {
            cell1.table.color = color;
        } else if (servos.curBarabanPos == BARABAN_CELL2_POS) {
            cell2.table.color = color;
        }
        checkNumberOfArtifacts();
    }

    public double getNextBarabanPos() {

        if(servos.curBarabanPos == BARABAN_CELL2_POS)
        {
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
        else if (servos.curBarabanPos == BARABAN_CELL1_POS)
        {
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

        switch (targCell){
            case 0:
                return BARABAN_CELL0_POS;
            case 1:
                return BARABAN_CELL1_POS;
            default:
                return BARABAN_CELL2_POS;
        }
    }
    public double findNeededArtifactPos(int color){
        if(cell2.table.color == color){
            return cell2.table.pos;
        }
        if(cell1.table.color == color){
            return cell1.table.pos;
        }

        if(cell0.table.color == color){
            return cell0.table.pos;
        }

        if(cell2.table.color != 0) return cell2.table.pos;
        else if(cell1.table.color != 0) return cell1.table.pos;
        else return cell0.table.pos;
    }
    public void deleteColorFromCell(){
        findNeededCell().table.color = 0;
        checkNumberOfArtifacts();
    }
    public DigitalCellsClass.Cell findNeededCell(){
        return cell0.table.pos == servos.curBarabanPos ? cell0 : (cell1.table.pos == servos.curBarabanPos ? cell1 : cell2);
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
        telemetry.addData("Cells", "C0:%s C1:%s C2:%s", getColorFromNumber(cell0.table.color),getColorFromNumber(cell1.table.color),getColorFromNumber(cell2.table.color));
        telemetry.addLine();
    }
}
