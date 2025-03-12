package frc.robot.commands.intake;

import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class ToDispenser extends FunctionWrapper {
    /**
     * Moves coral to dispenser. Will exit if elevator not in intake position. Only the intake
     * is used as a command requirement
     *
     * @param intake   the intake subsystem
     * @param elevator the elevator subsystem
     */
    public ToDispenser(Intake intake, Elevator elevator) {
        super(
            () -> intake.set(IntakeConstants.Power.kIntake),
            () -> !intake.data().broken || !elevator.atLevel(0), // if not L0, don't do this
            intake::stop,
            intake
        );
    }
}
