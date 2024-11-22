// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.ma5951.utils.ControlledMotors.AutoTuners;

// import com.ma5951.utils.ControlledMotors.VelocityControlledTalonFX;
// import com.ma5951.utils.DashBoard.MAShuffleboard;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutoKpTune extends Command {

//   private VelocityControlledTalonFX motor;
//   private MAShuffleboard board;
//   private Timer timer;
//   private double iteration;
//   private double errorSum;
//   private double lastErrorSum;
//   private double setPoint;

//   public AutoKpTune(int ID, double KpStart, double kv, double setpoint) {
//     board = new MAShuffleboard("AutoTuneMotor" + ID);
//     motor = new VelocityControlledTalonFX(7, true);
//     motor.setInverted(false);
//     motor.setNutralMode(true);
//     motor.setPID(KpStart, 0, 0);
//     motor.setFeedForward(0, kv);
//     setPoint = setpoint;

//     timer = new Timer();
//   }

//   public AutoKpTune(int ID, double KpStart, double setpoint) {
//     board = new MAShuffleboard("AutoTuneMotor" + ID);
//     motor = new VelocityControlledTalonFX(7, true);
//     motor.setInverted(false);
//     motor.setNutralMode(true);
//     motor.setPID(KpStart, 0, 0);
//     motor.setFeedForward(0, 0);
//     setPoint = setpoint;

//     timer = new Timer();
//   }

//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//     iteration = 0;
//     errorSum = 0;
//     lastErrorSum = 0;

//   }

//   @Override
//   public void execute() {
//     if (Math.round(timer.get()) == 5) {
//       timer.reset();
//       timer.start();

//       lastErrorSum = errorSum;
//       errorSum = 0;
//     }

//     errorSum += motor.getVelocity() - setPoint;

//   }

//   @Override
//   public void end(boolean interrupted) {
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
