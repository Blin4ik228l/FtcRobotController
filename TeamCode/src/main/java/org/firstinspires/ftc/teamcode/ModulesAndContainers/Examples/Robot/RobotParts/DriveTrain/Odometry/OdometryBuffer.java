package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry;

public class OdometryBuffer {
    private final OdometryData readBuffer = new OdometryData();
    private final OdometryData writeBuffer = new OdometryData();
    private final Object swapLock = new Object();

    // Для потока одометрии - получить буфер для записи
    public OdometryData beginWrite() {
        return writeBuffer;  // Пишем в writeBuffer
    }

    // Завершить запись и сделать данные доступными для чтения
    public void endWrite() {
        synchronized(swapLock) {
            // Копируем из writeBuffer в readBuffer
            readBuffer.set(writeBuffer);
        }
    }

    // Для главного потока - прочитать последние данные
    public OdometryData read() {
        return readBuffer;  // ТОЛЬКО ДЛЯ ЧТЕНИЯ!
    }
}
