package org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.Extenders2;

import org.firstinspires.ftc.teamcode.MainParts.Modules.DeviceTree.DeviceManager;

public abstract class UpdatableModule extends DeviceManager {
    public UpdatableModule(String searchingDevice) {
        super(searchingDevice);
    }
    public void update() {
        if(!isInitialized) return;
        else updateExt();
    }
    protected abstract void updateExt();
}
