package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.align.ReefAlign;
import frc.robot.constants.AlgaeManipulatorConstants;
import frc.robot.subsystems.elevator.Elevator;

public class ManipulatorElevate extends Command {
    final Elevator elevator;
    final ReefAlign align;

    boolean earlyExit = false;
    int level = -1;

    public ManipulatorElevate(Elevator elevator, ReefAlign align) {
        this.elevator = elevator;
        this.align = align;
    }

    @Override
    public void initialize() {
        // get (nullable) height without throwing an exception. prim int cannot be null.
        Integer height = AlgaeManipulatorConstants.kTagHeights.get(align.tagId());

        if (height == null) {
            earlyExit = true;
            return;
        }

        level = height;
        elevator.elevate(level);
    }

    @Override
    public boolean isFinished() {
        return earlyExit || elevator.atLevel(level);
    }
}
