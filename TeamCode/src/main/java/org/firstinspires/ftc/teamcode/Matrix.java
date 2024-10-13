package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import java.lang.reflect.Array;
import java.util.ArrayList;

// Класс для работы с матрицами
public class Matrix {
    Matrix(int rows, int cols){
        this.rows = rows;
        this.cols = cols;
        this.Mat = new double[rows][cols];
    }
    Matrix(Matrix other){
        this.rows = other.rows;
        this.cols = other.cols;
        this.Mat = new double[rows][cols];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                this.Mat[i][j] = other.Mat[i][j];
    }

    double [][] Mat;
    final int rows; // ряды
    final int cols; // столбцы

    // Сложить две матрицы и вернуть результат в виде новой матрицы
    Matrix add(Matrix other){
        Matrix resultMatrix = new Matrix(this);

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                resultMatrix.Mat[i][j] += other.Mat[i][j];

        return resultMatrix;
    }

    // Прибавить матрицу к объекту, в котором вызывался метод
    void increment(Matrix other){
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                this.Mat[i][j] += other.Mat[i][j];
    }

    // Умножить матрицу А на В
    static Matrix multiplyMatrix(@NonNull Matrix MatA, Matrix MatB){
        Matrix resultMat = new Matrix(MatB.rows, MatA.cols);

        for (int i = 0; i < resultMat.rows; i++) {
            for (int j = 0; j < resultMat.cols; j++) {
                for (int k = 0; k < MatA.cols; k++) {
                    resultMat.Mat[i][j] += MatA.Mat[i][k] * MatB.Mat[k][j];
                }
            }
        }

        return resultMat;
    }

    // Повернуть матрицу на угол в радианах
    Matrix turn(double rad){
        Matrix turnMatrix = new Matrix(3,3);
        turnMatrix.Mat = new double[][]{{   Math.cos(rad),  -Math.sin(rad), 0   },
                                        {   Math.sin(rad),  Math.cos(rad),  0   },
                                        {   0,              0,              1   }};
        return multiplyMatrix(turnMatrix, this);
    }

}
