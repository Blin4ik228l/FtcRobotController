package org.firstinspires.ftc.teamcode.Modules.Examples.Players.PL0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.CameraClass;
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
        switch (GeneralInformation.current.programName){
            case Auto:
                switch (semiAutoPlayerClass1.drivetrain.positionRobotController.generalState){
                    //Робот куда то едет
                    case EXECUTE_IN_PROCCESS:
                        switch (semiAutoPlayerClass1.drivetrain.positionRobotController.autoState) {
                            case CHECK_READINESS_FOR_START:
                                //Если выполнились все условия для старта
                                if (semiAutoPlayerClass1.drivetrain.positionRobotController.getCameraClass().onceSeen && semiAutoPlayerClass1.drivetrain.positionRobotController.getCameraClass().randomizeStatus == CameraClass.RandomizeStatus.Detected){
                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DONE;
                                }
                                switch (autoPlayerClass2.generalState){
                                    case LoadLogic:
                                        switch (autoPlayerClass2.loadState){
                                            case Idle:
                                                //Останавливаем сборщик в тот момент когда увидели все 3 предзагрузочных
                                                if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3)
                                                {
                                                    //Запоминаем состояния
                                                    generalState1 = AutoPlayerClass2.GeneralState.LoadLogic;
                                                    loadState = AutoPlayerClass2.LoadState.Idle;

                                                    //Останавливаем работу сборщика
                                                    autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
                                                }
                                                break;
                                        }
                                        break;
                                }
                                break;
                            case FIND_AND_GO_TO_FIRE_POS:
                                break;
                            case GO_AFORE_ARTIFACTS:
                                break;
                            //Переходим к сбору артефактов
                            case FIND_AND_GO_TO_ARTIFACTS:
                                switch (autoPlayerClass2.generalState){
                                    case Stop:
                                        if (generalState1 != null) autoPlayerClass2.generalState = generalState1;
                                        if (loadState != null) autoPlayerClass2.loadState = loadState;

                                        generalState1 = null;
                                        loadState = null;
                                        break;
                                    case LoadLogic:
                                        switch (autoPlayerClass2.loadState){
                                            case Idle:
                                                //Возможно программа считала "фантомный шар" но если он уже что то собрал переходим к сбору следующего
                                                if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3)
                                                {
                                                    generalState1 = AutoPlayerClass2.GeneralState.LoadLogic;
                                                    loadState = AutoPlayerClass2.LoadState.Idle;

                                                    //Останавливаем работу сборщика
                                                    autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;

                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DONE;
                                                }else {
                                                    //Запоминаем состояния
                                                    generalState1 = AutoPlayerClass2.GeneralState.LoadLogic;
                                                    loadState = AutoPlayerClass2.LoadState.Idle;

                                                    //Останавливаем работу сборщика
                                                    autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;

                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DONE;
                                                }

                                                break;
                                        }
                                        break;
                                    case FireLogic:
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
                                        semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                        semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_FIRE_POS;
                                        break;
                                    case LoadLogic:
                                        switch (autoPlayerClass2.loadState){
                                            case Find:
                                                //TODO поедет туда сюда
                                                semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.EMERGENCY_RATTLING;
                                                break;
                                            case Idle:
                                                if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3)
                                                {
                                                    //Запоминаем состояния
                                                    generalState1 = AutoPlayerClass2.GeneralState.LoadLogic;
                                                    loadState = AutoPlayerClass2.LoadState.Idle;

                                                    //Останавливаем работу сборщика
                                                    autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
                                                }
                                                break;
                                        }
                                        break;
                                    case FireLogic:
                                        break;
                                }
                                break;
                            case FIND_AND_GO_TO_FIRE_POS:
                                switch (autoPlayerClass2.generalState){
                                    case Stop:
                                        if (generalState1 != null) autoPlayerClass2.generalState = generalState1;
                                        if (loadState != null) autoPlayerClass2.loadState = loadState;

                                        generalState1 = null;
                                        loadState = null;
                                        break;
                                    case LoadLogic:
                                        break;
                                    case FireLogic:
                                        switch (autoPlayerClass2.fireState){
                                            case Fire:
                                                //TODO Если не можем разогнаться то подъедем ближе или дальше
                                                break;
                                            case Idle:
                                                if (semiAutoPlayerClass1.drivetrain.positionRobotController.shootedArtifacts == 9){} //Опускаем заслонку
                                                //Отстрелялись
                                                if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 0) {
                                                    //TODO Если что убрать
                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.GO_AFORE_ARTIFACTS;

                                                    generalState1 = autoPlayerClass2.generalState;
                                                    fireState = autoPlayerClass2.fireState;
                                                    autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
                                                }
                                                break;
                                        }
                                        break;
                                }
                                break;
                            case GO_AFORE_ARTIFACTS:
                                switch (autoPlayerClass2.generalState){
                                    case Stop:
                                        if (generalState1 != null) autoPlayerClass2.generalState = generalState1;
                                        if (fireState != null) autoPlayerClass2.fireState = fireState;

                                        generalState1 = null;
                                        fireState = null;

                                        semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                        semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_ARTIFACTS;
                                        break;
                                    case LoadLogic:
                                        switch (autoPlayerClass2.loadState){
                                            case Idle:
//                                                if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3)
//                                                {
//                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
//                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_FIRE_POS;
//                                                }else {
//                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
//                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_ARTIFACTS;
//                                                }
                                                break;
                                        }
                                        break;
                                    case FireLogic:
                                        switch (autoPlayerClass2.fireState){
                                            case Fire:
                                                //TODO Если не можем разогнаться то подъедем ближе или дальше
                                                break;
                                        }
                                        break;
                                }
                                break;
                            case FIND_AND_GO_TO_ARTIFACTS:
                                switch (autoPlayerClass2.generalState){
                                    case Stop:
                                        semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DELETE_ARTIFACT;
                                        break;
                                    case LoadLogic:
                                        switch (autoPlayerClass2.loadState){
                                            case Find:
                                                //TODO что делать если потеряли
                                                //Пробуем ехать к другому
                                                semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.DELETE_ARTIFACT;
                                                break;
                                            case Idle:
                                                //Убираем артефакт из общего массива данных
                                                if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3)
                                                {
                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_FIRE_POS;
                                                }
                                                break;
                                        }
                                        break;
                                    case FireLogic:
                                        break;
                                }
                                break;

                        }
                        break;
                    case DELETE_ARTIFACT:
                        semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                        semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_ARTIFACTS;
                        break;
                    case EMERGENCY_RATTLING:
                        switch (autoPlayerClass2.generalState){
                            //Возвращаем программу в исходное состояние
                            case Stop:
                                semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_FIRE_POS;
                                break;
                            case LoadLogic:
                                switch (autoPlayerClass2.loadState){
                                    case Idle:
                                        if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3)
                                        {
                                            //Запоминаем состояния
                                            generalState1 = AutoPlayerClass2.GeneralState.LoadLogic;
                                            loadState = AutoPlayerClass2.LoadState.Idle;

                                            //Останавливаем работу сборщика
                                            autoPlayerClass2.generalState = AutoPlayerClass2.GeneralState.Stop;
                                        }
                                        break;
                                }
                                break;
                            case FireLogic:
                                break;
                        }
                        break;
                }

                break;
            case TeleOp:
                switch (semiAutoPlayerClass1.drivetrain.positionRobotController.generalState){
                    case EXECUTE_IN_PROCCESS:
                        switch (semiAutoPlayerClass1.drivetrain.positionRobotController.autoState){
                            case FIND_AND_GO_TO_FIRE_POS:
                                break;
                            case GO_TO_LOAD_POS:
                                break;
                        }
                        break;
                    case DONE:
                        switch (semiAutoPlayerClass1.drivetrain.positionRobotController.autoState){
                            case FIND_AND_GO_TO_FIRE_POS:
                                switch (autoPlayerClass2.generalState){
                                    case FireLogic:
                                        switch (autoPlayerClass2.fireState){
                                            case Idle:
                                                if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 0){
                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                                    semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.GO_TO_LOAD_POS;
                                                }
                                                break;
                                        }
                                        break;
                                }
                                break;
                            case GO_TO_LOAD_POS:
                                semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.STOP;
                                break;
                        }
                        break;
                    case STOP:
                        switch (autoPlayerClass2.generalState){
                            case LoadLogic:
                                switch (autoPlayerClass2.loadState){
                                    case Load:
                                        if (autoPlayerClass2.collector.digitalCellsClass.getArtifactCount() == 3){
                                            semiAutoPlayerClass1.drivetrain.positionRobotController.generalState = PositionRobotController.GeneralState.Get_pos;
                                            semiAutoPlayerClass1.drivetrain.positionRobotController.autoState = PositionRobotController.AutoState.FIND_AND_GO_TO_FIRE_POS;
                                        }
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
        telemetry.addLine();
        telemetry.addLine("=PositionRobotController=");
        telemetry.addData("GeneralState", semiAutoPlayerClass1.drivetrain.positionRobotController.generalState.toString());
        telemetry.addData("AutoState", semiAutoPlayerClass1.drivetrain.positionRobotController.autoState.toString());
        telemetry.addLine();
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
