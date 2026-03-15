package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry;

public class OdometryBuffer{
    private OdometryData readBuffer;
    private final OdometryData writeBuffer;
    public OdometryBuffer(){
        readBuffer = new OdometryData();
        writeBuffer = new OdometryData();
    }
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
    public void endWrite2() {
        synchronized(swapLock) {
            // Копируем из writeBuffer в readBuffer
            readBuffer.add(writeBuffer);
        }
    }

    // Для главного потока - прочитать последние данные
    public OdometryData read() {
        return readBuffer;  // ТОЛЬКО ДЛЯ ЧТЕНИЯ!
    }
    public OdometryData read2() {
        OdometryData savedData = new OdometryData(readBuffer);
        readBuffer.reset();
        return savedData;
    }
}
