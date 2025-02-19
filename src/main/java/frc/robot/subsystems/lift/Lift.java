package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robotio.scoring.DispenserIO;
import frc.robotio.scoring.ElevatorIO;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;

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
