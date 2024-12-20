
package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  

  private RobotContainer m_robotContainer;
  private boolean isTeleop = false;
  public static boolean isStartingPose = false;
  private LoggedPose2d simulationPose2d;
  private TalonFX motor;
  private TalonFXSimState sim;
  private double x = 0;

  @Override
  public void robotInit() {
    MALog.getInstance();
    m_robotContainer = new RobotContainer();
    simulationPose2d = new LoggedPose2d("/Simulation/Pose");
    motor = new TalonFX(44);
    sim =motor.getSimState();
    sim.Orientation = ChassisReference.Clockwise_Positive;
    PoseEstimator.getInstance();

    MALog.getInstance().startLog();



  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    
    
  }

  @Override
  public void disabledInit() {
    if (isTeleop) {
      MALog.getInstance().stopLog();
    }
  }

  @Override
  public void disabledPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      }

    isTeleop = true;
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().addDriveTrainSimulation(SwerveConstants.SWERVE_DRIVE_SIMULATION);
  }

  @Override
  public void simulationPeriodic() {
    //SimulatedArena.getInstance().simulationPeriodic();
    //simulationPose2d.update(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose()) ;
    sim.setSupplyVoltage(12);

    System.out.println(motor.getVelocity().refresh().getValueAsDouble());

    //motor.setVoltage(11);
    sim.setRawRotorPosition(x);
    sim.setRotorVelocity(5);

    //System.out.println(x);

    x += 0.1;


  }

}
