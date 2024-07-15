// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Vision;

/** Add your docs here. */
public class CameraTypes {
    public enum CameraType {
        LLG3(1280, 800, 86 ),
        LL3(1280, 960, 70),
        LL2_PLUS(640, 480, 50);
    
        private final int width;
        private final int height;
        private final int fov;
    
        CameraType(int width, int height, int fov) {
            this.width = width;
            this.height = height;
            this.fov = fov;
        }
    
        public int getWidth() {
            return width;
        }
    
        public int getHeight() {
            return height;
        }
    
        public int getFov() {
            return fov;
        }
    }
    
}
