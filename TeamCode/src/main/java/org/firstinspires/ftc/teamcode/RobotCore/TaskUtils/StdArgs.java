package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

import org.firstinspires.ftc.teamcode.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.Utils.Position;

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
public class StdArgs {

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

    public static class driveStdArgs extends StdArgs {
        public driveStdArgs(Position position, double max_linear_speed, double max_angular_speed){
            this.position = position;
            this.max_linear_speed = max_linear_speed;
            this.max_angular_speed = max_angular_speed;
        }
        public driveStdArgs(Position position, double max_linear_speed) {
            this.position = position;
            this.max_linear_speed = max_linear_speed;
            this.max_angular_speed = CONSTS.MAX_RAD_PER_SEC;
        }
        public driveStdArgs(Position position) {
            this.position = position;
            this.max_linear_speed = CONSTS.MAX_CM_PER_SEC;
            this.max_angular_speed = CONSTS.MAX_RAD_PER_SEC;
        }
        public Position position;
        public double max_linear_speed;
        public double max_angular_speed;
    }

    public static class teleskopeStdArgs extends StdArgs {
        public teleskopeStdArgs(int goalPos, double max_speed){
            this.goalPos = goalPos;
            this.max_speed = max_speed;
        }
        public int goalPos;
        public double max_speed;
    }

}
