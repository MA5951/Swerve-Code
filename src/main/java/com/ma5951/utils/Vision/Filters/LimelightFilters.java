// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Linear Velocity
//Angular Velocity
//In field and not in specefied boxes
//Flickering pose 1. stationery flickring 2. distance from odometry (only auto)
//not deafult pose


//Is Flickering detection 

package com.ma5951.utils.Vision.Filters;

import com.ma5951.utils.Vision.Limelights.Limelight3G;

/** Add your docs here. */
public class LimelightFilters {
    private Limelight3G limelight;
    private LimelightFiltersConfig config;

    public LimelightFilters(Limelight3G camera , LimelightFiltersConfig configuration) {
        limelight = camera;
        config = configuration;
    }

    public boolean isValidForReset() {
        return limelight.isTarget();
    }


    
}
