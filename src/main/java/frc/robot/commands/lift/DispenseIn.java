package frc.robot.commands.lift;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.subsystems.lift.Lift;

public class DispenseIn extends FunctionWrapper {
    /**
     * Run the dispenser using intake
     *
     * @param lift the lift subsystem
     */
    public DispenseIn(Lift lift) {
        super(
            lift::dispense,
            () -> !lift.atLevel(0), // if we are not L0, don't intake, exit immediately
            () -> lift.dispense(0), // stop the dispenser on exit
            lift
        );
    }
}
