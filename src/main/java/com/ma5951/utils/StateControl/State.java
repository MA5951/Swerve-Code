// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class State{

    private Command statCommand;
    private Supplier<Boolean> stateCondition;
    private String stateName;

    public State(Supplier<Boolean> condition , Command command , String name) {
        statCommand = command;
        stateCondition = condition;
        stateName = name;
    }

    public State( Command command , String name) {
        statCommand = command;
        stateCondition = () -> false;
        stateName = name;
    }

    public boolean isStateRuning() {
        return statCommand.isScheduled();
    }

    public String getStateName() {
        return stateName;
    }

    public Command startState() {
        return new InstantCommand(() -> statCommand.schedule());
    }

    public void stopState() {
        statCommand.cancel();
    }

    public void runState() {
        if (stateCondition.get()) {
            statCommand.schedule();
        } else {
            statCommand.cancel();
        }
    }

}
