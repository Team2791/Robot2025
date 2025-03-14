package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.align.ReefAlign;
import frc.robot.subsystems.algae.AlgaeManipulator;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class FullManipulate extends ParallelRaceGroup {
    public FullManipulate(AlgaeManipulator manipulator, Drivetrain drivetrain, Elevator elevator) {
        final ReefAlign align = new ReefAlign(drivetrain, 0);

        addCommands(
            new RunManipulator(manipulator), // will exit once seq. group exits!
            new SequentialCommandGroup(
                align,
                new ManipulatorElevate(elevator, align),
                new WaitCommand(0.25)
            )
        );
    }
}
