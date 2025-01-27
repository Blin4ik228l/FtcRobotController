package org.firstinspires.ftc.teamcode.FieldView.FieldPosCore;

public class FieldHandler {

    public void createAndLoadAllCellsInField() {
        Character[] Alphabetical = {'A', 'B', 'C', 'D', 'E', 'F'};
        Character[] Numberious = {'1', '2', '3', '4', '5', '6'};

        for (int GY = 0; GY < Alphabetical.length; GY++) {
            for (int GX = 0; GX < Numberious.length; GX++) {
                field.setCellInField(GX, GY, new Cell(Alphabetical[GY], Numberious[GX]));
            }
        }
    }

    private int[] convertCharToXYCoord(Character Alphabetical, Character Numberious){
        int[] coord = new int[2];

        int X = Numberious - 48;
        int Y = Alphabetical - 64;

        coord[0] = X;
        coord[1] = Y;

        return coord;
    }

    Field field;

    public void init() {
        field = new Field(6, 6);
        createAndLoadAllCellsInField();
        field.calculateCellCoordinate();
    }

    private class Field {

        public Field(int cellsX, int cellsY) {
            this.cellsX = cellsX;
            this.cellsY = cellsY;

            initField();
        }

        private final int cellsX;
        private final int cellsY;

        private Cell[][] allCellsOnField;
        private final int width = 366;
        private final int height = 366;

        private double field[][];

        double startingGlobalPoseX = 0;
        double startingGlobalPoseY = 0;

        private void calculateCellCoordinate() {
            int X = -1;
            int Y = -1;

            for (int GY = 0; GY < cellsY; GY++) {
                for (int GX = 0; GX < cellsX; GX++) {
                    Cell cell = allCellsOnField[GX][GY];
                    for (double y = (cell.height * GY); y < cell.height * (1 + GY); y++) {
                        Y++;
                        for (double x = (cell.width * GX); x < cell.width * (1 + GX) ; x++) {
                            X++;
                            cell.globalXYCoordPlentyOfDots[X][Y] = new Dot(startingGlobalPoseX + x, startingGlobalPoseY + y, cell);
                        }
                        X = -1;
                    }
                    Y = -1;
                }
            }
        }
        public Cell getCellFromCharCoord(Character Alphabetical, Character Numberious){
            int X = convertCharToXYCoord( Alphabetical,  Numberious)[0];
            int Y = convertCharToXYCoord( Alphabetical,  Numberious)[1];

            return allCellsOnField[X][Y];
        }

        public Cell getCellFromXYCoord(double X, double Y){
            Cell testCell = new Cell('N', 'E');

            int widthCell = testCell.width;
            int heightCell = testCell.height;

            int XCellOnField = -1;
            int YCellOnField = -1;

            int multi;
            int last_multi;

            multi = 1;
            last_multi = 0;
            do {
                if(widthCell * multi > X && widthCell * last_multi <= X){
                    XCellOnField = last_multi;}
                multi++;
                last_multi++;
            }while (XCellOnField == -1);

            multi = 1;
            last_multi = 0;

            do {
                if(heightCell * multi > Y && heightCell * last_multi <= Y){
                    YCellOnField = last_multi;}
                multi++;
                last_multi++;
            }while (YCellOnField == -1);

            if(X == Y && X == 0){
                XCellOnField = 0;
                YCellOnField = 0;
            }

            return allCellsOnField[XCellOnField][YCellOnField];
        }

        public void setCellInField(int GX, int GY, Cell cell) {
            allCellsOnField[GX][GY] = cell;
        }

        public void initField() {
            field = new double[width][height];
            allCellsOnField = new Cell[cellsX][cellsY];
        }

        public double getArea() {
            return width * height;
        }
    }

    private class Cell {
        public Cell(Character alphabeticalCoord, Character numberCoord) {
            this.alphabeticalCoord = alphabeticalCoord;
            this.numberCoord = numberCoord;

            initCell();

            charCoord[0] = alphabeticalCoord;
            charCoord[1] = numberCoord;
        }

        public void initCell() {
            cell = new double[width][height];
            globalXYCoordPlentyOfDots = new Dot[height][width];
            charCoord = new Character[2];
        }

        private double cell[][];

        private final int width = 61 ;
        private final int height = 61 ;

        private Character[] charCoord;

        private final Character alphabeticalCoord;
        private final Character numberCoord;
        public Dot findDot(double x, double y){
            Dot findSomeDot = new Dot();

            for (int X = 0; X < width; X++) {
                for (int Y = 0; Y < height; Y++) {
                    if((int) globalXYCoordPlentyOfDots[X][Y].x == (int)x && (int)globalXYCoordPlentyOfDots[X][Y].y == (int)y){
                        findSomeDot = globalXYCoordPlentyOfDots[X][Y];
                    }
                }
            }
            return findSomeDot;
        }
        public Dot findDot2(double x, double y){
            int middleIndexX = (width - 1) / 2;
            int middleIndexY = (height - 1) / 2;

            int lastIndexX = width - 1;
            int lastIndexY = height - 1;

            int startIndexX = 0;
            int startIndexY = 0;

            while (globalXYCoordPlentyOfDots[middleIndexX][middleIndexY].x != x && globalXYCoordPlentyOfDots[middleIndexX][middleIndexY].y != y ) {
                middleIndexX = startIndexX + (lastIndexX - startIndexX) / 2;

                middleIndexY = startIndexY + (lastIndexY - startIndexY) / 2;

                if (globalXYCoordPlentyOfDots[middleIndexX][middleIndexY].x < x) {
                    startIndexX = middleIndexX + 1;
                } else if (globalXYCoordPlentyOfDots[middleIndexX][middleIndexY].x > x) {
                    lastIndexX = middleIndexX - 1;
                } else {
                    middleIndexX /= 1;
                }

                if (globalXYCoordPlentyOfDots[middleIndexX][middleIndexY].y < y) {
                    startIndexY = middleIndexY + 1;
                } else if (globalXYCoordPlentyOfDots[middleIndexX][middleIndexY].y > y) {
                    lastIndexY = middleIndexY - 1;
                } else {
                    middleIndexY /= 1;
                }
            }
            return globalXYCoordPlentyOfDots[middleIndexX][middleIndexY];
        }
        public double getArea() {
            return width * height;
        }
        Dot[][] globalXYCoordPlentyOfDots;

        public Character[] getCoordString() {
            return charCoord;
        }

        public String getName(){
            return  "{" + alphabeticalCoord + numberCoord +"}";
        }
    }

    private class Dot {
        public Dot(double x, double y, Cell parent) {
            this.x = x;
            this.y = y;
            this.parent = parent;

            coord[0] = x;
            coord[1] = y;
        }

        public Dot() {
            this.x = 999;
            this.y = 999;
            this.parent = new Cell('N', 'E');
        }

        public Dot(double x, double y) {
            this.x = (int) x;
            this.y = (int) y;
        }
        Cell parent;

        double coord[] = new double[2];
        double x;
        double y;

        public String getParent(){
            return "{"+parent.alphabeticalCoord + parent.numberCoord+"}";
        }
    }

    public void printAllFieldCells(FieldHandler fieldHandler) {
        for (int GY = 0; GY < fieldHandler.field.cellsY; GY++) {
            System.out.println();
            for (int GX = 0; GX < fieldHandler.field.cellsX; GX++) {
                System.out.print("[" + fieldHandler.field.allCellsOnField[GY][GX].charCoord[0] + fieldHandler.field.allCellsOnField[GY][GX].charCoord[1] + "]");
            }
        }
    }

    public void printAllFieldDotsXYOptimized(FieldHandler fieldHandler) {

        for (int GY = 0; GY < fieldHandler.field.cellsY; GY++) {
            for (int GX = 0; GX < fieldHandler.field.cellsX; GX++) {
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    if (y == fieldHandler.field.allCellsOnField[GY][GX].width / 2) {
                        System.out.print("___{" + fieldHandler.field.allCellsOnField[GY][GX].charCoord[0] + fieldHandler.field.allCellsOnField[GY][GX].charCoord[1] + "}___");
                    }
                    System.out.print("||||||||||||||||");
                }
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    System.out.print("-----------------");
                }
                for (int x = 0; x < fieldHandler.field.allCellsOnField[GY][GX].height; x++) {
                    System.out.println();
                    System.out.print("|");
                    for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                        System.out.print(" [" + fieldHandler.field.allCellsOnField[GY][GX].globalXYCoordPlentyOfDots[x][y].coord[0] + "]" +
                                " [" + fieldHandler.field.allCellsOnField[GY][GX].globalXYCoordPlentyOfDots[x][y].coord[1] + "]");
                    }
                    System.out.print("|");
                }
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    System.out.print("-----------------");
                }
            }
        }
    }
    public void printAllFieldDotsXY(FieldHandler fieldHandler) {

        for (int GY = 0; GY < fieldHandler.field.cellsY; GY++) {
            for (int GX = 0; GX < fieldHandler.field.cellsX; GX++) {
                double dotNumber = 0;
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    if (y == fieldHandler.field.allCellsOnField[GY][GX].width / 2) {
                        System.out.print("___{" + fieldHandler.field.allCellsOnField[GY][GX].charCoord[0] + fieldHandler.field.allCellsOnField[GY][GX].charCoord[1] + "}___");
                    }
                    System.out.print("||||||||||||||||");
                }
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    System.out.print("-----------------");
                }
                for (int x = 0; x < fieldHandler.field.allCellsOnField[GY][GX].height; x++) {
                    System.out.println();
                    System.out.print("|");
                    for (int k = 0; k < fieldHandler.field.allCellsOnField[GY][GX].width; k++) {
                        System.out.print("        {D: " + dotNumber + "}");
                        dotNumber++;
                    }
                    System.out.println();
                    System.out.print("|");
                    for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                        System.out.print("[ X: [" + fieldHandler.field.allCellsOnField[GY][GX].globalXYCoordPlentyOfDots[x][y].coord[0] + "]" +
                                " Y:[" + fieldHandler.field.allCellsOnField[GY][GX].globalXYCoordPlentyOfDots[x][y].coord[1] + "] ]");
                    }
                    System.out.print("|");
                }
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    System.out.print("-----------------");
                }
            }
        }
    }

    public void printAllFieldDots(FieldHandler fieldHandler) {

        for (int GY = 0; GY < fieldHandler.field.cellsY; GY++) {
            for (int GX = 0; GX < fieldHandler.field.cellsX; GX++) {
                double dotNumber = 0;
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    if (y == fieldHandler.field.allCellsOnField[GY][GX].width / 2) {
                        System.out.print("___{" + fieldHandler.field.allCellsOnField[GY][GX].charCoord[0] + fieldHandler.field.allCellsOnField[GY][GX].charCoord[1] + "}___");
                    }
                    System.out.print("||||||||||||||||");
                }
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    System.out.print("-----------------");
                }
                for (int x = 0; x < fieldHandler.field.allCellsOnField[GY][GX].height; x++) {
                    System.out.println();
                    System.out.print("|");
                    for (int k = 0; k < fieldHandler.field.allCellsOnField[GY][GX].width; k++) {
                        System.out.print(" {D: " + dotNumber + "}");
                        dotNumber++;
                    }
                }
                System.out.println();
                for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                    System.out.print("-----------------");
                }
            }
        }
    }

    public void printCellDots(FieldHandler fieldHandler, int GX, int GY) {
        double dotNumber = 0;
        System.out.println();
        for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
            if (y == fieldHandler.field.allCellsOnField[GY][GX].width / 2) {
                System.out.print("___{" + fieldHandler.field.allCellsOnField[GY][GX].charCoord[0] + fieldHandler.field.allCellsOnField[GY][GX].charCoord[1] + "}___");
            }
            System.out.print("||||||||||||||||");
        }
        System.out.println();
        for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
            System.out.print("-----------------");
        }
        for (int x = 0; x < fieldHandler.field.allCellsOnField[GY][GX].height; x++) {
            System.out.println();
            System.out.print("|");
            for (int k = 0; k < fieldHandler.field.allCellsOnField[GY][GX].width; k++) {
                System.out.print("        {D: " + dotNumber + "}");
                dotNumber++;
            }
            System.out.println();
            System.out.print("|");
            for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
                System.out.print("[ X: [" + fieldHandler.field.allCellsOnField[GY][GX].globalXYCoordPlentyOfDots[x][y].coord[0] + "]" +
                        " Y:[" + fieldHandler.field.allCellsOnField[GY][GX].globalXYCoordPlentyOfDots[x][y].coord[1] + "] ]");
            }
            System.out.print("|");
        }
        System.out.println();
        for (int y = 0; y < fieldHandler.field.allCellsOnField[GY][GX].width; y++) {
            System.out.print("-----------------");
        }
    }
}
