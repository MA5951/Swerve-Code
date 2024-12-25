
package frc.robot.Subsystem.Intake.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim extends IntakeIOReal{

    private TalonFXMotorSim motorSim;

    public IntakeIOSim() {
        super();
        motorSim = new TalonFXMotorSim(intakeMotor, motorConfig, DCMotor.getKrakenX60(1), 0.002);

    }

    @Override
    public void updatePeriodic() {
        motorSim.updateSim();
        super.updatePeriodic();
    }

}