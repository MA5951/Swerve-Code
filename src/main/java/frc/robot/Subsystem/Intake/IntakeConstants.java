// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IOs.IntakeIOReal;
import frc.robot.Subsystem.Intake.IOs.IntakeIOSim;

/** Add your docs here. */
public class IntakeConstants {

    public static final double INTAKE_POWER = 8;
    public static final double EJECT_POWER = -6;
    
    public static final double Gear = 1;
    
    public static final double PeakCurrentLimit = 30; 
    public static final double ContinuesCurrentLimit = 25; 
    public static final double PeakCurrentTime = 0.1; 
    public static final boolean IsCurrentLimitEnabled = true; 

    public static final State IDLE = StatesConstants.IDLE;
    public static final State INTAKING = new State("INTAKING");
    public static final State EJECTING = new State("EJECTING");

    public static final State[] SYSTEM_STATES = {IDLE, INTAKING, EJECTING};






}