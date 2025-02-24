package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks;

public class TasksGroupBuilder {
    public TasksGroupBuilder(TargetOfTasks targetOfThisGroup, int reward, OrdinaryTask...ordinaryTasks){

    }

    public OrdinaryTask[] ordinaryTasks;
    public TargetOfTasks target;
    public int reward;

    public enum TargetOfTasks {
        Sample_under_the_busket,
        Sample_first_lvl_busket,
        Sample_second_lvl_busket,
        Parcking,
        Sample_first_pipe_lvl,
        Sample_second_pipe_lvl,
        Nothing
    }

}
