package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Enums.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MainModule;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;

public abstract class ExecutorModule extends MainModule {
    protected ArrayList<Builder> builderPrograms;
    protected String executorName;
    protected ElapsedTime updateTime;
    public ExecutorModule(MainFile mainFile) {
        super(mainFile);
        programState = ProgramState.Waiting_For_Start;
        updateTime = new ElapsedTime();
    }
    protected int iterationCount = 1;
    protected double hz;
    public ProgramState programState;
    protected void resetTimer(){
        hz = 1 / updateTime.seconds();
        updateTime.reset();
    }

    public ProgramState execute(){
        executeExt();
        increaseIteration();
        resetTimer();
        return programState;
    };

    public void increaseIteration() {
        iterationCount++;
    }
    protected abstract void executeExt();
    public void showUpdateFreq(){
        telemetry.addData("Update time/hz", hz);
    }
    public void sayState(){
        telemetry.addLine(programState.name());
    }
    @Override
    public void sayModuleName() {
        telemetry.addLine( "-----" + this.getClass().getSimpleName().toUpperCase() + "-----");
    }
    @Override
    public void showData() {
        sayModuleName();
        showDataExt();
        sayState();
        showUpdateFreq();
        sayLastWords();
    }

    public abstract void executeTeleOp();
    public abstract void executeAuto();
    @Override
    protected void sayLastWords() {
        telemetry.addLine("-----------------------------------");
    }

    public static class Builder {
        public String name;
        //Как бы перемещаем каретку
        private ExecutorModule activeProgram;
        private Deque<ExecutorModule> programs = new ArrayDeque<>();
        private Deque<ExecutorModule> programsCopy = new ArrayDeque<>();
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
            for (ExecutorModule state : programs) {
                if (state.executorName == string) activeProgram = state; break;
            }
            return this;
        }
        public void switchActiveProgramToStart(){
            activeProgram = programs.getFirst();
        }
        public Builder addProgram(ExecutorModule program){
            programs.addLast(program);
            activeProgram = programs.getFirst();
            return this;
        }
        public boolean isLastProgramWasExecuted(){
            return activeProgram == null;
        }
    }
}
