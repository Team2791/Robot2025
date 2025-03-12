package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

public class Elevator extends SubsystemBase {
    final ElevatorIO elevator;

    public Elevator(ElevatorIO elevator) {
        this.elevator = elevator;
    }

    /** Level is 0 for intake, [1, 4] for [L1, L4] scoring */
    public void elevate(int level) {
        assert level >= 0 && level <= 4 : "Invalid level: " + level + " (expected [0, 4])";
        double height = ElevatorConstants.Heights.kLevels[level];
        elevator.setDesiredPosition(Meters.of(height));
    }

    /** Elevate but add an offset to prevent manual slamming */
    public void manualElevate(boolean up) {
        double height = ElevatorConstants.Heights.kLevels[up ? 4 : 0];
        double offset = height + 0.02 * (up ? -1 : 1);
        elevator.setDesiredPosition(Meters.of(offset));
    }

    /** Manual controls: hold elevator */
    public void hold(double offset) {
        double current = elevator.data.height().in(Meters);
        double setpoint = MathUtil.clamp(
            current + offset,
            ElevatorConstants.Heights.kIntake,
            ElevatorConstants.Heights.kL4
        );

        elevator.setDesiredPosition(Meters.of(setpoint));
    }

    /** Outputs [0, 4], where 0 is intake, and decimals are in-between */
    @AutoLogOutput(key = "Elevator/Level")
    public double level() {
        double height = elevator.data.height().in(Meters);
        for (int i = 0; i < ElevatorConstants.Heights.kLevels.length - 1; i++) {
            if (height < ElevatorConstants.Heights.kLevels[i + 1]) {
                double relative = height - ElevatorConstants.Heights.kLevels[i];
                double range = ElevatorConstants.Heights.kLevels[i + 1] - ElevatorConstants.Heights.kLevels[i];
                return i + relative / range;
            }
        }
        return 4;
    }

    /** Check if at level */
    public boolean atLevel(int level) {
        assert level >= 0 && level <= 4 : "Invalid level: " + level + " (expected [0, 4])";
        double height = elevator.data.height().in(Meters);
        double levelHeight = ElevatorConstants.Heights.kLevels[level];
        double error = Math.abs(height - levelHeight);
        return error < ElevatorConstants.Heights.kTolerance;
    }

    /** Get a copy of elevator data */
    public ElevatorIO.ElevatorData data() {
        return this.elevator.data.clone();
    }

    @Override
    public void periodic() {
        elevator.update();
        Logger.processInputs("Elevator", elevator.data);
    }
}
