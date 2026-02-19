package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.GameState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.ProgramState;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Reason;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public abstract class MainSystem extends UpdatableModule {
    public GeneralInformation generalInformation;
    public SemiAutoPlayerClass1 semiAutoPlayerClass1;
    public AutoPlayerClass2 autoPlayerClass2;
    public RobotClass robotClass;

    public Thread pl1, pl2, rob;
    public MainSystem(GeneralInformation generalInformation, RobotClass robotClass, OpMode op){
        super(op);
        this.generalInformation = generalInformation;
        this.generalInformation.gameState = GameState.Fire;
        this.semiAutoPlayerClass1 = new SemiAutoPlayerClass1(generalInformation, robotClass, op);
        this.autoPlayerClass2 = new AutoPlayerClass2(generalInformation, robotClass, op);
        this.robotClass = robotClass;
    }

    public void startExecuting(){
        int targetFrequencyHz = 50;  // 50 раз в секунду
        long targetPeriodNs = 1_000_000_000 / targetFrequencyHz;  // 20,000,000 нс
        long targetSleepMs = targetPeriodNs / 1_000_000;  // 20 мс


        Runnable player1 = new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted())
                {
                    long start = System.nanoTime();
                    semiAutoPlayerClass1.execute();

                    long elapsed = System.nanoTime() - start;
                    long sleepTime = targetPeriodNs - elapsed;

                    if (sleepTime > 0) {
                        try {
                            Thread.sleep(targetSleepMs);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }
            }
        };
        Runnable player2 = new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted())
                {
                    long start = System.nanoTime();
                    autoPlayerClass2.execute();

                    long elapsed = System.nanoTime() - start;
                    long sleepTime = targetPeriodNs - elapsed;

                    if (sleepTime > 0) {
                        try {
                            Thread.sleep(targetSleepMs);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }
            }
        };
        Runnable robot = new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted())
                {
                    long start = System.nanoTime();
                    robotClass.update();

                    long elapsed = System.nanoTime() - start;
                    long sleepTime = targetPeriodNs - elapsed;

                    if (sleepTime > 0) {
                        try {
                            Thread.sleep(targetSleepMs);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }

            }
        };

        pl1 = new Thread(player1);
        pl2 = new Thread(player2);
        rob = new Thread(robot);

        pl1.start();
        pl2.start();
        rob.start();
    }
    public void update() {
        switch (generalInformation.programName){
            case TeleOp:
                break;
            default:
                boolean isPlayer1Finished = false;
                boolean isPlayer2Finished = false;

               switch (generalInformation.gameState){
                   case Load:
                       switch (semiAutoPlayerClass1.programState){
                           case Executing:
                               break;
                           case Finished:
                               isPlayer1Finished = true;
                               break;
                       }
                       switch (autoPlayerClass2.programState){
                           case Executing:
                               break;
                           case Finished:
                               isPlayer2Finished = true;
                               break;
                       }

                       if (isPlayer1Finished && isPlayer2Finished){
                           //прерываем работу 2 игрока чтобы он не начал стрелять пока едет
                           autoPlayerClass2.isInterrupted = true;
                           generalInformation.gameState = GameState.Fire;
                       } else if (isPlayer1Finished && !isPlayer2Finished) {
                           //Если 1 игрок доехал до точки, но 2 игрок ещё не собрал 3 артефакта -> едем к следующему
                           semiAutoPlayerClass1.positionController.switchPos();
                       } else if (isPlayer2Finished && !isPlayer1Finished) {
                           //Если 2 игрок уже собрал (каким то образом 3 артефакта, когда 1 игрок не доехал до след артефакта), то едем стрелять, и прерываем работу 2 игрока чтобы он не начал стрелять пока едет
                           autoPlayerClass2.isInterrupted = true;
                           generalInformation.gameState = GameState.Fire;
                       }
                       break;
                   default:
                       switch (semiAutoPlayerClass1.programState){
                           case Executing:
                               break;
                           case Finished:
                               isPlayer1Finished = true;
                               break;

                       }
                       switch (autoPlayerClass2.programState){
                           case Executing:
                               break;
                           case Finished:
                               isPlayer2Finished = true;
                               break;
                       }
                       //тут самое главное подъехать к точке стрельбы, а потом в игру вступит 2 игрок
                       if (isPlayer1Finished && isPlayer2Finished){
                           //Закончили цикл стрельбы -> едем собирать артефакты
                           generalInformation.gameState = GameState.Load;
                       } else if (isPlayer1Finished && !isPlayer2Finished) {
                           //возвращаем в строй 2 игрока
                           autoPlayerClass2.isInterrupted = false;
                       }
                       break;
               }
               break;
        }


    }

    @Override
    public void showData() {
        telemetry.addLine("===MAIN SYSTEM===");


        telemetry.addLine();
    }


}
