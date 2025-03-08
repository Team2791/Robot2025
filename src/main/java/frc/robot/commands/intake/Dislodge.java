package frc.robot.commands.intake;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.DispenserConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.intake.Intake;

public class Dislodge extends FunctionWrapper {
    /**
     * Reverse until the beam brake is freed
     *
     * @param intake    the intake subsystem
     * @param dispenser the dispenser subsystem
     */
    public Dislodge(Intake intake, Dispenser dispenser) {
        super(
            () -> {
                intake.set(IntakeConstants.Power.kDislodge);
                dispenser.dispense(DispenserConstants.Power.kDislodge);
            },
            () -> !intake.getRoller().broken,
            () -> {
                intake.stop();
                dispenser.dispense(0);
            },
            intake,
            dispenser
        );
    }
}
