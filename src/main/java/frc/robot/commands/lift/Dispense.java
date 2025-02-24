package frc.robot.commands.lift;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.subsystems.lift.Lift;
import frc.robot.util.TimeoutUtil;

/**
 * Run the dispenser
 */
public class Dispense extends FunctionWrapper {
    public Dispense(Lift lift, boolean intake) {
        super(
            lift::dispense,
            () -> lift.getDispenser().broken == intake,
            () -> TimeoutUtil.setTimeout(() -> lift.dispense(0), 1000), // delay: mismatch between brake stop & dispense
            lift
        );
    }
}
