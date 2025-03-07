package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.logging.Alerter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.photon.Photon;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;

import static frc.robot.constants.MathConstants.kTau;

public class ReefAlign extends Command {
    final Drivetrain drivetrain;
    final Photon photon;
    final CommandXboxController driverctl;
    final double offset;

    final PIDController xController = new PIDController(
        ControlConstants.Align.kOrthoP,
        ControlConstants.Align.kOrthoI,
        ControlConstants.Align.kOrthoD
    );

    final PIDController yController = new PIDController(
        ControlConstants.Align.kOrthoP,
        ControlConstants.Align.kOrthoI,
        ControlConstants.Align.kOrthoD
    );

    final PIDController rotController = new PIDController(
        ControlConstants.Align.kTurnP,
        ControlConstants.Align.kTurnI,
        ControlConstants.Align.kTurnD
    );

    Pose2d tagPose = null;
    Pose2d targetPose = null;

    /**
     * Align to the reef using photon vision, positioning self to score on a particular branch
     *
     * @param drivetrain the drivetrain subsystem
     * @param photon     the photon subsystem
     * @param driverctl  the driver controller
     * @param direction  the direction to align to, can be -1 for right or 1 for left
     */
    public ReefAlign(Drivetrain drivetrain, Photon photon, CommandXboxController driverctl, int direction) {
        assert Math.abs(direction) == 1 : "wanted direction = +-1; got " + direction;

        this.drivetrain = drivetrain;
        this.driverctl = driverctl;
        this.photon = photon;
        this.offset = VisionConstants.Align.kReefOffset * direction;
        rotController.enableContinuousInput(0, kTau);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // reset
        tagPose = null;
        targetPose = null;

        Pose2d robotPose = drivetrain.getPose();

        // get list of reef apriltag poses
        List<Integer> targetIds = VisionConstants.AprilTag.reef();
        List<Pose2d> targetPoses = targetIds.stream()
            .map(VisionConstants.AprilTag.kLayout::getTagPose)
            .filter(Optional::isPresent) // this shouldn't happen
            .map(p -> p.get().toPose2d())
            .toList();

        // get reef tag closest to robot
        tagPose = robotPose.nearest(targetPoses);

        // offset target robot pose by 1/2 of bumper, and align to either left or right
        targetPose = tagPose.transformBy(
            new Transform2d(
                0.5 * RobotConstants.Drivetrain.kBumperLength,
                offset,
                Rotation2d.kPi
            )
        );

        if (robotPose.getTranslation().getDistance(targetPose.getTranslation()) >= 2.0) {
            xController.setSetpoint(robotPose.getX());
            yController.setSetpoint(robotPose.getY());
            rotController.setSetpoint(robotPose.getRotation().getRadians());
            return;
        }

        // set pid controller setpoints
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rotController.setSetpoint(targetPose.getRotation().getRadians());

        // record to akit
        Logger.recordOutput("ReefAlign/Target", targetPose);
        Logger.recordOutput("ReefAlign/Tag", tagPose);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        // pid
        double xPower = xController.calculate(currentPose.getX());
        double yPower = yController.calculate(currentPose.getY());
        double rotPower = rotController.calculate(currentPose.getRotation().getRadians());

        drivetrain.drive(xPower, yPower, rotPower);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void end(boolean interrupted) {
        Alerter.getInstance().rumble();
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }
}
