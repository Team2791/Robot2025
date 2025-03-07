package frc.robot.commands.intake;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lift.Lift;

public class ToDispenser extends FunctionWrapper {
    /**
     * Moves coral to dispenser. Will exit if lift not in intake position. Only the intake
     * is used as a command requirement
     *
     * @param intake the intake subsystem
     * @param lift   the lift subsystem
     */
    public ToDispenser(Intake intake, Lift lift) {
        super(
            () -> intake.set(IntakeConstants.Power.kIntake),
            () -> !intake.getRoller().broken || !lift.atLevel(0), // if not L0, don't do this
            intake::stop,
            intake
        );
    }
}
