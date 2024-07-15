package frc.robot;

public class PortMap {

  public static class CanBus {
    public static final String CANivoreBus = "Swerve";
    public static final String RioBus = "rio";
  }

  public static class Controllers {
    public static final int driveID = 0;
    public static final int operatorID = 1;    
  }

  public static class Swerve {
    public static final int leftFrontAbsoluteEncoder = 22;
    public static final int leftFrontDriveID = 8;
    public static final int leftFrontTurningID = 5;

    public static final int leftBackAbsoluteEncoder = 21;
    public static final int leftBackDriveID = 4;
    public static final int leftBackTurningID = 9;

    public static final int rightFrontAbsoluteEncoder = 23;
    public static final int rightFrontDriveID = 7;
    public static final int rightFrontTurningID = 6;

    public static final int rightBackAbsoluteEncoder = 24;
    public static final int rightBackDriveID = 2;
    public static final int rightBackTurningID = 3;

    public static final int Pigeon2ID = 12;
  }

  public static class Shooter {
    public static final int masterID = 0;
    public static final int angleMasterID = 0;

    public static final int absoluteEncoderID = 0;

  }

  public static class Conveyr {
    public static final int masterID = 0;
    public static final int upperSensorID = 0;
    public static final int lowerSensorID = 0;
  }

  public static class LED {
    public static final int ledPort = 9; 
  }

  public static class Vision {
    public static final int absEncoderID = 0;
    public static final int masterID = 0;
  }
}