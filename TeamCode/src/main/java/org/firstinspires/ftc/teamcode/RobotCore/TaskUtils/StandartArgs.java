package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

/**
 * Класс, в котором описывается структура аргументов для методов-обработчиков задач
 * При создании новых видов аргументов, обьязательно наследуйте их от Args, не забудьте
 * создать конструктор класса.
 * Как написать свой метод-обработчик задачи?
 * Метод, обрабатывающий задачу должен иметь вид:
 * private int мойМетодОбработчик(Args _args){
 *     Args.мойКлассАргументов args = (Args.мойКлассАргументов)_args
 *     ...
 *     return result; // переменная result должна содержать 0, если задача завершена, иначе -1
 * }
 * в методе уже можете пользоваться переменной args как хотите
 */
public class StandartArgs {

    // Пример:
    // public static class мойКлассАргументов extends Args {
    //     мойКлассАргументов(тип_1 переменная_1, ..., тип_n переменная_m) {
    //         this.переменная_1 = переменная_1;
    //         ...
    //         this.переменная_m = переменная_m;
    //     }
    //     тип_1 переменная_1;
    //     ...
    //     тип_n переменная_m;
    // }

    public static class driveArgs extends StandartArgs {
        public driveArgs(Position position, double max_linear_speed, double max_angular_speed, double delayTime){
            this.position = position;
            this.max_linear_speed = max_linear_speed;
            this.max_angular_speed = max_angular_speed;
            this.delayTime = delayTime;
        }
        public driveArgs(Position position, double max_linear_speed, double delayTime) {
            this.position = position;
            this.max_linear_speed = max_linear_speed;
            this.max_angular_speed = Consts.MAX_ANGULAR_SPEED;
        }
        public driveArgs(Position position, double delayTime) {
            this.position = position;
            this.max_linear_speed = Consts.MAX_LINEAR_SPEED;
            this.max_angular_speed = Consts.MAX_ANGULAR_SPEED;
        }
        public Position position;
        public double max_linear_speed;
        public double max_angular_speed;
        public double delayTime;
    }

    public static class verticalArgs extends StandartArgs {
        public verticalArgs(double teleskope_height,double max_speed, double delayTime){
            this.teleskope_height = teleskope_height;
            this.max_speed = max_speed;
            this.delayTime = delayTime;
        }
        public double teleskope_height;
        public double max_speed;
        public double delayTime;
    }


    public static class horizontalArgs extends StandartArgs {
        public horizontalArgs( double servo_pos, double delayTime){
            this.servo_pos = servo_pos;
            this.delayTime = delayTime;
        }
        public double servo_pos;
        public double delayTime;
    }

    public static class captureArgs extends StandartArgs{
        public captureArgs(double flipPos, double hookPos, double delayTime){
            this.flipPos = flipPos;
            this.hookPos = hookPos;
            this.delayTime = delayTime;
        }
        public double flipPos;
        public double hookPos;
        public double delayTime;
    }

    public static class robotSleep extends StandartArgs{
        public robotSleep(double time){
            this.time = time;
        }
        public double time;
    }
    public static class doWhile extends StandartArgs{
        public doWhile(double power){
            this.power = power;
        }
       public double power;
    }

    public static class selfInspect extends StandartArgs{
        public selfInspect(){

        }
    }
}

