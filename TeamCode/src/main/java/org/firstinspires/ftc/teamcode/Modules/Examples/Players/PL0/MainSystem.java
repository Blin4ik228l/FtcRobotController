package org.firstinspires.ftc.teamcode.Modules.Examples.Players.PL0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.PositionRobotController;

public class MainSystem extends ExecutableModule {
    public SemiAutoPlayerClass1 semiAutoPlayerClass1;
    public AutoPlayerClass2 autoPlayerClass2;
    public MainSystem(SemiAutoPlayerClass1 semiAutoPlayerClass1, AutoPlayerClass2 autoPlayerClass2, OpMode op){
        super(op);
        this.semiAutoPlayerClass1 = semiAutoPlayerClass1;
        this.autoPlayerClass2 = autoPlayerClass2;

        AutoPlayerClass2.GeneralState generalState1 = null;
        AutoPlayerClass2.LoadState loadState = null;
        AutoPlayerClass2.FireState fireState = null;
        AutoPlayerClass2.AnotherStates anotherStates = null;
        PositionRobotController.AutoState autoState = null;
        PositionRobotController.GeneralState generalState2 = null;
    }
    public AutoPlayerClass2.GeneralState generalState1;
    public AutoPlayerClass2.LoadState loadState;
    public AutoPlayerClass2.FireState fireState;
    public AutoPlayerClass2.AnotherStates anotherStates;
    public PositionRobotController.AutoState autoState;
    public PositionRobotController.GeneralState generalState2;
    @Override
    public void execute() {
        //Смотрим на состояние сборщика, если что отключаем какого то из игроков
        switch (semiAutoPlayerClass1.drivetrain.positionRobotController.generalState){
            //Робот куда то едет
            case EXECUTE_IN_PROCCESS:
                switch (semiAutoPlayerClass1.drivetrain.positionRobotController.autoState) {
                    case CHECK_READINESS_FOR_START:
                        switch (autoPlayerClass2.generalState){
                            case Stop:
                                break;
                            case LoadLogic:
                                break;
                            case FireLogic:
                                switch (autoPlayerClass2.fireState){
                                    case Fire:
                                        //Запоминаем состояния
                                        generalState1 = AutoPlayerClass2.GeneralState.FireLogic;
                                        fireState = AutoPlayerClass2.FireState.Fire;

                                        //Останавливаем работу сборщика
                                        autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
                                        break;
                                }
                                break;
                        }
                        break;
                    case FIND_AND_GO_TO_FIRE_POS:
                        switch (autoPlayerClass2.generalState){
                            case Stop:
                                break;
                            case LoadLogic:
                                switch (autoPlayerClass2.loadState){
                                    case Find:
                                        break;
                                    case Load:
                                        //Сборщик возможно до сих пор не увидел все 3 шара, пока бездействуем
                                        break;
                                    case Idle:
                                        break;
                                }
                                break;
                            case FireLogic:
                                switch (autoPlayerClass2.fireState){
                                    case Fire:
                                        //Останавливаем пргограмму
                                        generalState1 = AutoPlayerClass2.GeneralState.FireLogic;
                                        fireState = AutoPlayerClass2.FireState.Fire;

                                        autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
                                        break;
                                }
                                break;
                        }
                        break;
                        //Переходим к сбору артефактов
                    case FIND_AND_GO_TO_ARTIFACTS:
                        switch (autoPlayerClass2.generalState){
                            case Stop:
                                break;
                            case LoadLogic:
                                switch (loadState){
                                    case Idle:
                                        //Возможно программа считала "фантомный шар" но если он уже что то собрал переходим к сбору следующего
                                        semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DONE;
                                        break;
                                }
                                break;
                            case FireLogic:
                                switch (autoPlayerClass2.fireState){
                                    case Fire:
                                        break;
                                }
                                break;
                        }
                        break;
                }
                break;
                // Робот остановился
            case DONE:
                switch (semiAutoPlayerClass1.drivetrain.positionRobotController.autoState) {
                    case CHECK_READINESS_FOR_START:
                        //Считываем предзагрузочные
                        switch (autoPlayerClass2.generalState){
                            //Возвращаем программу в исходное состояние
                            case Stop:
                                if (generalState1 != null) autoPlayerClass2.generalState = generalState1;
                                if (fireState != null) autoPlayerClass2.fireState = fireState;

                                generalState1 = null;
                                fireState = null;

                                semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                break;
                            case LoadLogic:
                                switch (autoPlayerClass2.loadState){
                                    case Idle:
                                        if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3) semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                        break;
                                }
                                break;
                            case FireLogic:
                                switch (autoPlayerClass2.fireState){
                                    case Fire:
                                        generalState1 = AutoPlayerClass2.GeneralState.FireLogic;
                                        fireState = AutoPlayerClass2.FireState.Fire;

                                        autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
                                        break;
                                }
                                break;
                        }
                        break;
                    case FIND_AND_GO_TO_FIRE_POS:
                        switch (autoPlayerClass2.generalState){
                            case Stop:
                                if (generalState1 != null) autoPlayerClass2.generalState = generalState1;
                                if (fireState != null) autoPlayerClass2.fireState = fireState;

                                generalState1 = null;
                                fireState = null;
                                break;
                            case LoadLogic:
                                switch (autoPlayerClass2.loadState){
                                    case Find:
                                        //TODO устанавливаем вариант езды туда сюда чтобы датчик цвета увидел предзагрузочный
                                        break;
                                    case Idle:
                                        break;
                                }
                                break;
                            case FireLogic:
                                switch (autoPlayerClass2.fireState){
                                    case Fire:
                                        //TODO Если не можем разогнаться то подъедем ближе или дальше
                                        break;
                                    case Idle:
                                        if (semiAutoPlayerClass1.drivetrain.positionRobotController.shootedArtifacts == 9) //Опускаем заслонку
                                        //Отстрелялись
                                        if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 0) semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                        break;
                                }
                                break;
                        }
                        break;
                    case FIND_AND_GO_TO_ARTIFACTS:
                        switch (autoPlayerClass2.generalState){
                            case Stop:
                                break;
                            case LoadLogic:
                                switch (loadState){
                                    case Find:
                                        //TODO что делать если потеряли
                                        semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DELETE_ARTIFACT;
                                        break;
                                    case Idle:
                                        //Убираем артефакт из общего массива данных
                                        semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DELETE_ARTIFACT;
                                        if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3) semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_FIRE_POS;
                                        break;
                                }
                                break;
                            case FireLogic:
                                switch (autoPlayerClass2.fireState){
                                    case Fire:
                                        break;
                                }
                                break;
                        }
                        break;
                }
                break;
        }

    }

    @Override
    public void showData() {
        telemetry.addLine("===MAIN SYSTEM===");
        telemetry.addLine("=PositionRobotController=");
        telemetry.addData("GeneralState", semiAutoPlayerClass1.drivetrain.positionRobotController.generalState.toString());
        telemetry.addData("AutoState", semiAutoPlayerClass1.drivetrain.positionRobotController.autoState.toString());
        telemetry.addLine("=AutoPlayerClass2=");
        telemetry.addData("GeneralState", autoPlayerClass2.generalState.toString());

        if(autoPlayerClass2.generalState == AutoPlayerClass2.GeneralState.LoadLogic){
            telemetry.addData("LoadState", autoPlayerClass2.loadState.toString());
        }else {
            telemetry.addData("FireState", autoPlayerClass2.fireState.toString());
            telemetry.addData("AnotherStates", autoPlayerClass2.anotherStates.toString());
        }

        telemetry.addLine();
    }
}
