package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeManipulator;

public class RunManipulator extends Command {
    final AlgaeManipulator manipulator;

    public RunManipulator(AlgaeManipulator manipulator) {
        this.manipulator = manipulator;
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.toggle(true);
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.toggle(false);
    }
}
