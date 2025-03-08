package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lift.Lift;

public class ManualElevate extends Command {
    final Lift lift;
    final boolean up;

    /**
     * Manually move elevator. - for up, + for down
     *
     * @param lift the lift subsystem
     * @param up   whether to move up
     */
    public ManualElevate(Lift lift, boolean up) {
        this.lift = lift;
        this.up = up;
    }

    @Override
    public void initialize() {
        lift.manualElevate(up);
    }

    @Override
    public void end(boolean interrupted) {
        lift.hold(0.1 * (up ? 1 : -1));
    }
}
