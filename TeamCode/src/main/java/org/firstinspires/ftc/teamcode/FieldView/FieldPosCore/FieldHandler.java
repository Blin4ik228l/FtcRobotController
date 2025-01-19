package org.firstinspires.ftc.teamcode.FieldView.FieldPosCore;

import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

import java.util.ArrayList;

public class FieldHandler {

    Field field;

    public void init(){
        field = new Field();
        field.initField();
        createAndLoadAllCellsInField();
    }

    public void createAndLoadAllCellsInField(){
        String[] Alphabetical = {"A", "B", "C", "D", "E", "F"};
        String[] Numberious = {"1", "2", "3", "4", "5", "6"};

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                field.setCellInField(new Cell(Alphabetical[i], Numberious[j]));
            }
        }
    }

    private class Field{

        public Field(){

        }

        private ArrayList<Cell> allCellsOnField;

        private final int width = 366;
        private final int height = 366;
        private double field[][];

        private void calculateCellCoordinate(){
            for (int i = 0; i < 36; i++) {

            }
        }

        public void setCellInField(Cell cell){
            allCellsOnField.add(cell);
        }

        public void initField(){
            field = new double[width][height];
            allCellsOnField = new ArrayList<Cell>();
        }

    }

    private class Cell{

        public Cell(String alphabeticalCoord, String numberCoord){
            this.alphabeticalCoord = alphabeticalCoord;
            this.numberCoord = numberCoord;

            initCell();

            coord[0] = alphabeticalCoord;
            coord[1] = numberCoord;
        }

        private double cell[][];

        private final int width = 61;
        private final int height = 61;

        private Position posUpperLineCell;
        private Position posDownLineCell;

        private String[] coord = new String[2];

        private String alphabeticalCoord;
        private String numberCoord;

        public String[] getCoordString(){
            return coord;
        }

        public void setPosition(Position posDownLineCell, Position posUpperLineCell){
            this.posDownLineCell = posDownLineCell;
            this.posUpperLineCell = posUpperLineCell;
        }

        public void initCell(){
            cell = new double[width][height];
        }
    }
}
