package frc.robot.commands.dispenser;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.DispenserConstants;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.elevator.Elevator;

public class DispenseOut extends ParallelDeadlineGroup {
    /**
     * Dispense the coral if not at L0
     *
     * @param dispenser the dispenser subsystem
     * @param elevator  the elevator subsystem. not a command requirement
     */
    public DispenseOut(Dispenser dispenser, Elevator elevator) {
        super(
            new WaitCommand(0.75),
            new FunctionWrapper(
                () -> dispenser.dispense(
                    elevator.atLevel(1)
                        ? DispenserConstants.Power.kDispenseL1
                        : DispenserConstants.Power.kDispense
                ),
                () -> elevator.atLevel(0), // if at L0, return immediately
                () -> dispenser.dispense(0),
                dispenser
            )
        );
    }
}
