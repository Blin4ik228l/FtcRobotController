package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Matrix {
    double [][] Mat;
    int rows;//ряды
    int cols;//столбцы
    Matrix(int rows, int cols){
        this.Mat = new double[rows][cols];

    }

    Matrix transformMat(@NonNull Matrix mat){
        Matrix resultMat = new Matrix(mat.rows, this.cols);

        for (int i = 0; i < resultMat.rows; i++) {
            for (int j = 0; j < resultMat.cols; j++) {
                resultMat.Mat[i][j] = 0;
                for (int k = 0; k < this.cols; k++) {
                    resultMat.Mat[i][j] += Mat[i][k] * mat.Mat[k][j];
                }
            }
        }
        return resultMat;
    }
}
