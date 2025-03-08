package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DispenserConstants;
import frc.robot.subsystems.lift.Lift;

public class DispenseIn extends Command {
    final Lift lift;
    boolean wasBroken = false;

    /**
     * Run the dispenser using intake
     *
     * @param lift the lift subsystem
     */
    public DispenseIn(Lift lift) {
        this.lift = lift;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        wasBroken = false;
        lift.dispense(DispenserConstants.Power.kDispenseIn);
    }

    @Override
    public void execute() {
        wasBroken |= lift.getDispenser().broken;
    }

    @Override
    public void end(boolean interrupted) {
        lift.dispense(0);
    }

    @Override
    public boolean isFinished() {
        return !lift.atLevel(0) || (wasBroken && !lift.getDispenser().broken);
    }
}
