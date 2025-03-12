package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ManualElevate extends Command {
    final Elevator elevator;
    final boolean up;

    /**
     * Manually move elevator. - for up, + for down
     *
     * @param elevator the elevator subsystem
     * @param up       whether to move up
     */
    public ManualElevate(Elevator elevator, boolean up) {
        this.elevator = elevator;
        this.up = up;
    }

    @Override
    public void initialize() {
        elevator.manualElevate(up);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.hold(0.1 * (up ? 1 : -1));
    }
}
