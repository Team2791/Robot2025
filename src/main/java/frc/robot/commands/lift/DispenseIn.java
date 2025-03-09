package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DispenserConstants;
import frc.robot.subsystems.dispenser.Dispenser;
import frc.robot.subsystems.elevator.Elevator;

public class DispenseIn extends Command {
    final Dispenser dispenser;
    final Elevator elevator;

    boolean wasBroken = false;

    /**
     * Run the dispenser using intake
     *
     * @param lift     the lift subsystem
     * @param elevator the elevator subsystem, not a command requirement
     */
    public DispenseIn(Dispenser lift, Elevator elevator) {
        this.dispenser = lift;
        this.elevator = elevator;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        wasBroken = false;
        dispenser.dispense(DispenserConstants.Power.kDispenseIn);
    }

    @Override
    public void execute() {
        wasBroken |= dispenser.data().broken;
    }

    @Override
    public void end(boolean interrupted) {
        dispenser.dispense(0);
    }

    @Override
    public boolean isFinished() {
        return !elevator.atLevel(0) || (wasBroken && !dispenser.data().broken);
    }
}
