package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.util.FunctionWrapper;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroReplay;
import frc.robot.subsystems.drivetrain.gyro.NavX;
import frc.robot.subsystems.drivetrain.module.ModuleReplay;
import frc.robot.subsystems.drivetrain.module.ModuleSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeReplay;
import frc.robot.subsystems.intake.IntakeSpark;
import frc.robot.util.AdvantageUtil;
import frc.robot.util.Alerter;

public class RobotContainer {
    // controllers
    final CommandXboxController driverctl;
    final CommandXboxController operctl;

    // subsystems
    final Drivetrain drivetrain = new Drivetrain(
            AdvantageUtil.matchReal(
                    NavX::new,
                    () -> {
                        throw new UnsupportedOperationException("No intake simulation available");
                    },
                    GyroReplay::new),
            AdvantageUtil.matchReal(
                    ModuleSpark::new,
                    (a) -> {
                        throw new UnsupportedOperationException("No intake simulation available");
                    },
                    ModuleReplay::new));

    final Intake intake = new Intake(AdvantageUtil.matchReal(
            IntakeSpark::new,
            () -> {
                throw new UnsupportedOperationException("No intake simulation available");
            },
            IntakeReplay::new));

    public RobotContainer() {
        this.driverctl = new CommandXboxController(IOConstants.Controller.kDriver);
        this.operctl = new CommandXboxController(IOConstants.Controller.kOperator);

        configureBindings();

        Alerter.getInstance().provideControllers(driverctl, operctl);
        CameraServer.startAutomaticCapture();
        CameraServer.removeCamera("USB Camera 0"); // fix USB Camera 0 problem, but we still need to init CamServer
    }

    private void configureBindings() {
        Command joystickDrive = new RunCommand(() -> drivetrain.drive(driverctl), drivetrain);
        drivetrain.setDefaultCommand(joystickDrive);
        driverctl.start().onTrue(new FunctionWrapper(drivetrain::resetGyro).ignoringDisable(true));

        driverctl
                .leftBumper()
                .whileTrue(new FunctionWrapper(
                        () -> intake.intake(Intake.IntakeState.Intake),
                        () -> intake.intake(Intake.IntakeState.Hold),
                        intake));

        driverctl.rightBumper().onTrue(new FunctionWrapper(() -> intake.intake(Intake.IntakeState.Outtake), intake));

        driverctl
                .a()
                .whileTrue(new FunctionWrapper(
                        () -> intake.pivot(Intake.PivotState.Up), () -> intake.pivot(Intake.PivotState.Stop), intake));

        driverctl
                .y()
                .whileTrue(new FunctionWrapper(
                        () -> intake.pivot(Intake.PivotState.Down),
                        () -> intake.pivot(Intake.PivotState.Stop),
                        intake));
    }

    public Command getAutonomousCommand() {
        return Commands.parallel(
                Commands.deadline(
                        new WaitCommand(1.5), new RunCommand(() -> drivetrain.drive(0.25, 0.0, 0.0), drivetrain)),
                Commands.deadline(
                        new WaitCommand(4.5),
                        new FunctionWrapper(
                                () -> intake.pivot(Intake.PivotState.Down),
                                () -> intake.pivot(Intake.PivotState.Stop),
                                intake)));
    }
}
