package com.ma5951.utils.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class MALog {
    private static MALog logger;
    private NetworkTable table;
    private final NetworkTableInstance networkTable;
    

    public MALog() {
        networkTable = NetworkTableInstance.getDefault();
    }

    public NetworkTable getMainNT() {
        table = networkTable.getTable("/");
        return table;
    }

    public void startLog() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    public void stopLog() {
        DataLogManager.stop();
    }

    public static MALog getInstance() {
        if (logger == null) {
            logger = new MALog();
        }
        return logger;
    }
    
}
