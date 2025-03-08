package frc.robot.commands.intake;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.DispenserConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lift.Lift;

public class Dislodge extends FunctionWrapper {
    /**
     * Reverse until the beam brake is freed
     *
     * @param intake the intake subsystem
     */
    public Dislodge(Intake intake, Lift lift) {
        super(
            () -> {
                intake.set(IntakeConstants.Power.kDislodge);
                lift.dispense(DispenserConstants.Power.kDislodge);
            },
            () -> !intake.getRoller().broken,
            () -> {
                intake.stop();
                lift.dispense(0);
            },
            intake,
            lift
        );
    }
}
