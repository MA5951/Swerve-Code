package frc.robot;

public class PortMap {
  public static class Swerve {
    public static final int leftFrontAbsoluteEncoder = 22;
    public static final int leftFrontDriveID = 4;
    public static final int leftFrontTurningID = 5;

    public static final int leftBackAbsoluteEncoder = 23;
    public static final int leftBackDriveID = 7;
    public static final int leftBackTurningID = 6;

    public static final int rightFrontAbsoluteEncoder = 21;
    public static final int rightFrontDriveID = 8;
    public static final int rightFrontTurningID = 9;

    public static final int rightBackAbsoluteEncoder = 24;
    public static final int rightBackDriveID = 2;
    public static final int rightBackTurningID = 3;
  }

  public static class Controllers {
    public static final int driverJostick = 0;
  }
}