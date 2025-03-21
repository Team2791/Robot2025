package frc.robot.commands.intake;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class TakeIn extends FunctionWrapper {
    /**
     * Take in until beam brake is hit
     *
     * @param intake the intake subsystem
     */
    public TakeIn(Intake intake) {
        super(
            () -> intake.set(IntakeConstants.Power.kIntake),
            () -> intake.data().broken,
            intake::stop,
            intake
        );
    }
}
