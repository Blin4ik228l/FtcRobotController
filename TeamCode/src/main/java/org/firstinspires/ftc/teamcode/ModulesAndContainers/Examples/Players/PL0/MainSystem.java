package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.ExecutorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutingModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.ExecutableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

public class MainSystem extends ExecutorModule {
    protected GeneralInformation generalInformation;
    protected SemiAutoPlayerClass1 semiAutoPlayerClass1;
    protected AutoPlayerClass2 autoPlayerClass2;
    protected RobotClass robotClass;
    protected FileSystem fileSystem;
    protected ElapsedTime matchTimer;
    public ArrayList<Thread> threads;
    public MainSystem(MainFile mainFile){
        super(mainFile);
        this.generalInformation = mainFile.generalInformation;
        this.robotClass = new RobotClass(mainFile);

        this.semiAutoPlayerClass1 = new SemiAutoPlayerClass1(mainFile, robotClass);
        this.autoPlayerClass2 = new AutoPlayerClass2(mainFile, robotClass);

        this.fileSystem = new FileSystem(mainFile);

        this.matchTimer = new ElapsedTime();

        this.threads = new ArrayList<>();

        switch (generalInformation.programName){
            case TeleOp:
                fileSystem.loadLastPosition();
                break;
            default:
                break;
        }
    }

    @Override
    protected void executeExt() {
        switch (generalInformation.programName) {
            case TeleOp:
                if (TELEOP_SECONDS - matchTimer.seconds() < 5) {
                    generalInformation.gameTactick = GameTactick.Parking;
                }
                executeTeleOp();
//                fileSystem.update();
                break;
            default:
                if (AUTO_SECONDS - matchTimer.seconds() < 5) {
                }
                executeAuto();
                break;
        }

        robotClass.drivetrain.update();
        robotClass.hoodedShoter.update();
    }

    public void startExecuting(){
        createRunnable(semiAutoPlayerClass1).createRunnable(autoPlayerClass2).createRunnable(robotClass);

        for (Thread thread: threads) {
            thread.start();
        }

        matchTimer.reset();
    }
    public MainSystem createRunnable(ExecutorModule executorModule){
        int targetFrequencyHz = 50;
        long targetPeriodNs = 1_000_000_000 / targetFrequencyHz;
        long targetSleepMs = targetPeriodNs / 1_000_000;

        Runnable rn = new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted())
                {
                    long start = System.nanoTime();

                    executorModule.execute();

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
        threads.add(new Thread(rn));
        return this;
    }
    public double TELEOP_SECONDS = 120;
    public double AUTO_SECONDS = 30;
    public void interrupt(){
        for (Thread thread: threads) {
            thread.interrupt();
        }
        switch (generalInformation.programName){
            case TeleOp:
                fileSystem.writeAnother();
                fileSystem.deleteOdometry();
                break;
            default:
                fileSystem.writeOdometry();
                break;
        }

    }

    public void executeTeleOp(){
        boolean isPlayer1Finished = false;
        boolean isPlayer2Finished = false;
        switch (generalInformation.gameTactick){
            case Load:
                switch (autoPlayerClass2.programState){
                    case Executing:
                        break;
                    case Interrupted:
                        break;
                    case Finished:
                        isPlayer2Finished = true;
                        break;
                }

                if (isPlayer2Finished){
                    //прерываем работу 2 игрока чтобы он не начал стрелять пока едет
                    autoPlayerClass2.isInterrupted = true;
                    generalInformation.gameTactick = GameTactick.Fire;
                }
                break;
            case Fire:
                switch (semiAutoPlayerClass1.programState){
                    case Executing:
                        break;
                    case Interrupted:
                        break;
                    case Finished:
                        isPlayer1Finished = true;
                        break;

                }
                switch (autoPlayerClass2.programState){
                    case Executing:
                        break;
                    case Interrupted:
                        break;
                    case Finished:
                        isPlayer2Finished = true;
                        break;
                }
                //тут самое главное подъехать к точке стрельбы, а потом в игру вступит 2 игрок
                if (isPlayer1Finished && isPlayer2Finished){
                    //Закончили цикл стрельбы -> едем собирать артефакты
                    generalInformation.gameTactick = GameTactick.Load;
                } else if (isPlayer1Finished && !isPlayer2Finished) {
                    //возвращаем в строй 2 игрока
                    autoPlayerClass2.isInterrupted = false;
                }
                break;
            default:
                break;
        }
    }
    public void executeAuto(){
        boolean isPlayer1Finished = false;
        boolean isPlayer2Finished = false;
        switch (generalInformation.gameTactick){
            case Load:
                switch (semiAutoPlayerClass1.programState){
                    case Executing:
                        break;
                    case Interrupted:
                        break;
                    case Finished:
                        isPlayer1Finished = true;
                        break;
                }
                switch (autoPlayerClass2.programState){
                    case Executing:
                        break;
                    case Interrupted:
                        break;
                    case Finished:
                        isPlayer2Finished = true;
                        break;
                }

                if (isPlayer1Finished && isPlayer2Finished){
                    //прерываем работу 2 игрока чтобы он не начал стрелять пока едет
                    autoPlayerClass2.isInterrupted = true;
                    generalInformation.gameTactick = GameTactick.Fire;
                } else if (isPlayer1Finished && !isPlayer2Finished) {
                    //Если 1 игрок доехал до точки, но 2 игрок ещё не собрал 3 артефакта -> едем к следующему
                    semiAutoPlayerClass1.positionController.switchPos();
                } else if (isPlayer2Finished && !isPlayer1Finished) {
                    //Если 2 игрок уже собрал (каким то образом 3 артефакта, когда 1 игрок не доехал до след артефакта), то едем стрелять, и прерываем работу 2 игрока чтобы он не начал стрелять пока едет
                    autoPlayerClass2.isInterrupted = true;
                    generalInformation.gameTactick = GameTactick.Fire;
                }
                break;
            case Fire:
                switch (semiAutoPlayerClass1.programState){
                    case Executing:
                        break;
                    case Interrupted:
                        break;
                    case Finished:
                        isPlayer1Finished = true;
                        break;

                }
                switch (autoPlayerClass2.programState){
                    case Executing:
                        break;
                    case Interrupted:
                        break;
                    case Finished:
                        isPlayer2Finished = true;
                        break;
                }
                //тут самое главное подъехать к точке стрельбы, а потом в игру вступит 2 игрок
                if (isPlayer1Finished && isPlayer2Finished){
                    //Закончили цикл стрельбы -> едем собирать артефакты
                    generalInformation.gameTactick = GameTactick.Load;
                } else if (isPlayer1Finished && !isPlayer2Finished) {
                    //возвращаем в строй 2 игрока
                    autoPlayerClass2.isInterrupted = false;
                }
                break;
            default:
                break;
        }
    }

    @Override
    protected void showDataExt() {
        telemetry.addLine(generalInformation.telemetryPages.toString());
        switch (generalInformation.telemetryPages){
            case Show_all:
                semiAutoPlayerClass1.showData();
                autoPlayerClass2.showData();
                robotClass.showData();
                break;
            case Show_pl1:
                semiAutoPlayerClass1.showData();
                break;
            case Show_pl2:
                autoPlayerClass2.showData();
                break;
            case Show_robot:
                robotClass.showData();
                break;
            case Show_modules_freq:
                semiAutoPlayerClass1.showUpdateFreq();
                autoPlayerClass2.showUpdateFreq();
                robotClass.showUpdateFreq();
                break;
        }
    }



    public class FileSystem extends UpdatingModule {
        //Данному классу требуется доступ ко всем объектам программы
        private File odometryData, anotherData;
        private List<String> logBuffer;
        private Date date;
        public FileSystem(MainFile mainFile){
            super(mainFile);
            odometryData = AppUtil.getInstance().getSettingsFile("OdometryData.txt");
            anotherData = AppUtil.getInstance().getSettingsFile("AnotherData.txt");

            logBuffer = new ArrayList<>();
            date = new Date();
        }

        @Override
        protected void updateExt() {
            logBuffer.add(String.format("%.2f %.2f %.2f %.2f %s %n", Math.abs(autoPlayerClass2.trackEmulator.targHeadVel * RAD), Math.abs(robotClass.odometry.odometryBufferForTuret.read().getHeadVel() * RAD),
                    autoPlayerClass2.trackEmulator.targHead * RAD, robotClass.odometry.odometryBufferForTuret.read().getPosition().getHeading() * RAD, matchTimer.seconds()));
        }
        public void loadLastPosition() {
            OdometryData lastData = null;

            try (BufferedReader br = new BufferedReader(new FileReader(odometryData))) {
                String line;
                String lastLine = null;

                // Читаем до последней строки
                while ((line = br.readLine()) != null) {
                    lastLine = line;
                }

                if (lastLine != null) {
                    String[] parts = lastLine.split(",");
                    if (parts.length >= 4) {
                        double x = Double.parseDouble(parts[1]);
                        double y = Double.parseDouble(parts[2]);
                        double heading = Double.parseDouble(parts[3]);

                        Position2D pos = new Position2D(x, y, heading);
                        lastData = new OdometryData(pos, new Vector2(0), 0);

                        robotClass.odometry.setStartPos(pos);
                        telemetry.addData("Loaded", "X:%.1f Y:%.1f H:%.1f", x, y, Math.toDegrees(heading));
                    }
                }

            } catch (FileNotFoundException e) {
                telemetry.addData("File", "No save file yet");
            } catch (IOException | NumberFormatException e) {
                telemetry.addData("Load Error", e.getMessage());
            }
        }
        public void writeOdometry(){
            try (FileWriter fw = new FileWriter(odometryData, true); ) {
                OdometryData savedRobotData = robotClass.odometry.odometryBufferForRobot.read();
                List<String> odometryBuffer = new ArrayList<>();
                odometryBuffer.add(String.format("X", savedRobotData.getPosition().getX()));
                odometryBuffer.add(String.format("Y", savedRobotData.getPosition().getY()));
                odometryBuffer.add(String.format("Heading", savedRobotData.getPosition().getHeading()));
                for (String line : odometryBuffer) fw.write(line);
            }catch (IOException e) {
            }
        }
        public void writeAnother(){
            try (FileWriter fw = new FileWriter(anotherData, true); ) {
                for (String line : logBuffer) fw.write(line);
            }catch (IOException e) {
            }
        }
        public void deleteAnother(){
            anotherData.delete();
        }
        public void deleteOdometry(){
            odometryData.delete();
        }
        @Override
        public void showData() {

        }

        @Override
        protected void showDataExt() {

        }
    }
}
