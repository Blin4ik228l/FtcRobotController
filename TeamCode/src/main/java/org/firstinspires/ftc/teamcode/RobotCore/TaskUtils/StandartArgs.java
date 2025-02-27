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
        public driveArgs(Position position, double max_linear_speed, double max_angular_speed, long sleep){
            this.position = position;
            this.max_linear_speed = max_linear_speed;
            this.max_angular_speed = max_angular_speed;
            this.sleep = sleep;
        }
        public driveArgs(Position position, double max_linear_speed) {
            this.position = position;
            this.max_linear_speed = max_linear_speed;
            this.max_angular_speed = Consts.MAX_ANGULAR_SPEED;
        }
        public driveArgs(Position position) {
            this.position = position;
            this.max_linear_speed = Consts.MAX_LINEAR_SPEED;
            this.max_angular_speed = Consts.MAX_ANGULAR_SPEED;
        }
        public Position position;
        public double max_linear_speed;
        public double max_angular_speed;
        public long sleep;
    }

    public static class teleskopeArgs extends StandartArgs {
        public teleskopeArgs(double teleskope_height, double servo_pos, double max_speed){
            this.teleskope_height = teleskope_height;
            this.servo_pos = servo_pos;
            this.max_speed = max_speed;
        }
        public double teleskope_height;
        public double servo_pos;
        public double max_speed;
    }

    public static class captureArgs extends StandartArgs{
        public captureArgs(double flipPos, double hookPos){
            this.flipPos = flipPos;
            this.hookPos = hookPos;
        }
        public double flipPos;
        public double hookPos;
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

