package frc.robot.commands.lift;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.subsystems.lift.Lift;

public class DispenseOut extends FunctionWrapper {
    /**
     * Dispense the coral if not at L0
     *
     * @param lift the lift subsystem
     */
    public DispenseOut(Lift lift) {
        super(
            lift::dispense,
            () -> !lift.canDispense(), // if we can't dispense, return asap
            () -> lift.dispense(0),
            lift
        );
    }
}
