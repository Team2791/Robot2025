package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class Characterization extends SequentialCommandGroup {
    final Drivetrain drivetrain;
    final SysIdRoutine routine;

    final Timer timeout = new Timer();

    public Characterization(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drivetrain/SysId/State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> drivetrain.characterize(voltage.in(Volts)),
                null,
                this.drivetrain
            )
        );

        addCommands(
            Commands.run(() -> drivetrain.characterize(0), drivetrain)
                .withTimeout(1.0)
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
        );
    }
}
