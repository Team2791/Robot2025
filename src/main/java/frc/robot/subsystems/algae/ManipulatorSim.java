package frc.robot.subsystems.algae;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.AlgaeManipulatorConstants;

public class ManipulatorSim extends ManipulatorSpark {
    final SparkMaxSim turnSim;
    final SparkMaxSim spinSim;

    final DCMotorSim turnMotorSim;
    final DCMotorSim spinMotorSim;

    public ManipulatorSim() {
        super();

        DCMotor turnGearbox = DCMotor.getNEO(1);
        DCMotor spinGearbox = DCMotor.getNEO(1);

        turnSim = new SparkMaxSim(turn, turnGearbox);
        spinSim = new SparkMaxSim(spin, spinGearbox);

        turnMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                turnGearbox,
                AlgaeManipulatorConstants.TurnMotor.kReduction,
                AlgaeManipulatorConstants.TurnMotor.kMoI
            ),
            turnGearbox
        );

        spinMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                spinGearbox,
                AlgaeManipulatorConstants.SpinMotor.kReduction,
                AlgaeManipulatorConstants.SpinMotor.kMoI
            ),
            spinGearbox
        );
    }

    @Override
    public void update() {
        // update wpi mech
        turnMotorSim.setInput(turn.getAppliedOutput() * RoboRioSim.getVInVoltage());
        spinMotorSim.setInput(spin.getAppliedOutput() * RoboRioSim.getVInVoltage());
        turnMotorSim.update(0.02);
        spinMotorSim.update(0.02);

        // update rev sim
        turnSim.iterate(
            turnMotorSim.getAngularVelocityRadPerSec(),
            RoboRioSim.getVInVoltage(),
            0.02
        );

        spinSim.iterate(
            spinMotorSim.getAngularVelocityRadPerSec(),
            RoboRioSim.getVInVoltage(),
            0.02
        );

        // account for battery voltage drops
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                turnSim.getMotorCurrent(),
                spinSim.getMotorCurrent()
            )
        );

        // update data
        super.update();

        // not connected -- in sim
        this.data.turnConnected = false;
        this.data.spinConnected = false;
    }
}
