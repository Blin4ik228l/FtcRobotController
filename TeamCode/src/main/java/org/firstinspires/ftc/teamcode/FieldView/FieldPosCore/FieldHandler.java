package org.firstinspires.ftc.teamcode.FieldView.FieldPosCore;

import android.graphics.ImageFormat;
import android.media.ImageReader;
import android.os.Handler;


import java.io.File;

public class FieldHandler {
    int Field[][] = new int[366][366];
    File fieldIm = new File("/FieldView/Field/Field.png");
    ImageReader reader = ImageReader.newInstance(366, 366, ImageFormat.RGB_565, 1);
    ImageReader.OnImageAvailableListener listener;
    Handler handler;
    public void Math(){
        reader.setOnImageAvailableListener(listener, handler);
        listener.onImageAvailable(reader);
    }
}
