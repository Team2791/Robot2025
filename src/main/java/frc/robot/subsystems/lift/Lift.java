package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DispenserConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robotio.scoring.DispenserIO;
import frc.robotio.scoring.DispenserIO.DispenserData;
import frc.robotio.scoring.ElevatorIO;
import frc.robotio.scoring.ElevatorIO.ElevatorData;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class Lift extends SubsystemBase {
    final DispenserIO dispenser;
    final ElevatorIO elevator;

    public Lift(DispenserIO dispenser, ElevatorIO elevator) {
        this.dispenser = dispenser;
        this.elevator = elevator;
    }

    /** Level is 0 for intake, [1, 4] for [L1, L4] scoring */
    public void elevate(int level) {
        assert level >= 0 && level <= 4 : "Invalid level: " + level + " (expected [0, 4])";
        double height = ElevatorConstants.Heights.kLevels[level];
        elevator.setDesiredPosition(Meters.of(height));
    }

    /** Outputs [0, 4], where 0 is intake, and decimals are in-between */
    public double getLevel() {
        double height = elevator.data.position.in(Radians) * ElevatorConstants.Sprocket.kRadius;
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
        double height = elevator.data.position.in(Radians) * ElevatorConstants.Sprocket.kRadius;
        double levelHeight = ElevatorConstants.Heights.kLevels[level];
        double error = Math.abs(height - levelHeight);
        return error < ElevatorConstants.Heights.kTolerance;
    }

    /** Run the dispenser */
    public void dispense() {
        dispense(DispenserConstants.kDispense);
    }

    /** Run the dispenser (with power) */
    public void dispense(double power) {
        assert Math.abs(power) <= 1.0 : "Needed -1.0 <= power <= 1.0, got %f".formatted(power);
        dispenser.set(power);
    }

    /** Get a copy of the dispenser data */
    public DispenserData getDispenser() {
        return dispenser.data.clone();
    }

    /** Get a copy of the elevator data */
    public ElevatorData getElevator() {
        return elevator.data.clone();
    }

    @Override
    public void periodic() {
        // update the elevator and dispenser
        elevator.update();
        dispenser.update();

        // log
        Logger.processInputs("Lift/Dispenser", dispenser.data);
        Logger.processInputs("Lift/Elevator", elevator.data);
    }
}
