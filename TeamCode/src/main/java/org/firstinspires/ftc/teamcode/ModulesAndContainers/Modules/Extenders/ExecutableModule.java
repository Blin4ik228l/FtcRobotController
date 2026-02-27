package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;

public abstract class ExecutableModule extends Module {
    public ProgramState programState = ProgramState.Waiting_For_Start;
    protected ArrayList<Builder> builderPrograms;

    public ProgramState safeExecute(){
        ProgramState programState1;

        if(!isInitialized) programState1 = execute();

        else  programState1 = ProgramState.Executing;

        return programState1;
    }
    protected abstract ProgramState execute();
    public ExecutableModule(OpMode op, String name){
        super(op);
        this.name = name;
    }

    protected String name;
    public static class Builder {
        //Как бы перемещаем каретку
        public ProgramState programState;
        private ExecutableModule activeProgram;
        private Deque<ExecutableModule> programs = new ArrayDeque<>();
        private Deque<ExecutableModule> programsCopy = new ArrayDeque<>();
        public void switchActiveProgram(){
            programsCopy = new ArrayDeque<>(programs);

            while (!programsCopy.isEmpty() && programsCopy.peekFirst() != activeProgram){
                programsCopy.pollFirst();
            }

            if (programsCopy.peekFirst() != null){
                activeProgram = programsCopy.pollFirst();
            }
            else {
                //Решим это проблему выше
                activeProgram = null;
            }
        }
        public Builder switchActiveProgramTo(String string){
            for (ExecutableModule state : programs) {
                if (state.name == string) activeProgram = state; break;
            }
            return this;
        }
        public void switchActiveProgramToStart(){
            activeProgram = programs.getFirst();
        }
        public Builder addProgram(ExecutableModule program){
            programs.addLast(program);
            activeProgram = programs.getFirst();
            return this;
        }
        public boolean isLastProgramWasExecuted(){
            return activeProgram == null;
        }
        public ProgramState execute() {
            programState = activeProgram.execute();
            return programState;
        }
        public String getName(){
            return activeProgram.name;
        }
    }
}
