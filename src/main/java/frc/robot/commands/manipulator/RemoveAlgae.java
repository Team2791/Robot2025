package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.align.ReefAlign;
import frc.robot.subsystems.algae.AlgaeManipulator;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class RemoveAlgae extends SequentialCommandGroup {
    /**
     * Remove algae from the robot, aligning to the station and lifting the elevator based on apriltag
     *
     * @param manipulator the algae manipulator subsystem
     * @param drivetrain  the drivetrain subsystem
     * @param elevator    the elevator subsystem
     */
    public RemoveAlgae(AlgaeManipulator manipulator, Drivetrain drivetrain, Elevator elevator) {
        ReefAlign align = new ReefAlign(drivetrain, 0);

        addCommands(
            align,
            new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new ManipulatorElevate(elevator, align),
                new RunManipulator(manipulator)
            )
        );
    }
}
