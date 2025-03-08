package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Alerter;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public abstract class AlignClosest extends Command {
	final Drivetrain drivetrain;
	final Supplier<List<Pose2d>> targetPoses;
	final Transform2d offset;

	Pose2d tagPose = null;
	Pose2d targetPose = null;
	boolean earlyExit = false;

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

	public AlignClosest(Drivetrain drivetrain, Supplier<List<Integer>> targetIds, Transform2d offset) {
		this.drivetrain = drivetrain;
		this.offset = offset;
		this.targetPoses = () -> targetIds.get()
			.stream()
			.map(VisionConstants.AprilTag.kLayout::getTagPose)
			.filter(Optional::isPresent)
			.map(Optional::get)
			.map(Pose3d::toPose2d)
			.toList();

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		Pose2d robotPose = drivetrain.getPose();

		// get nearest tag
		tagPose = robotPose.nearest(targetPoses.get());
		targetPose = tagPose.transformBy(offset);

		if (robotPose.getTranslation().getDistance(targetPose.getTranslation()) >= VisionConstants.Align.kMaxDistance) {
			System.out.println("AlignClosest: Target too far away");
			earlyExit = true;
			return;
		}

		// set setpoints
		xController.setSetpoint(targetPose.getX());
		yController.setSetpoint(targetPose.getY());
		rotController.setSetpoint(targetPose.getRotation().getRadians());

		// record to akit
		Logger.recordOutput("Align/Target", targetPose);
		Logger.recordOutput("Align/Tag", tagPose);
	}

	@Override
	public void execute() {
		if (earlyExit) return;

		Pose2d currentPose = drivetrain.getPose();

		// pid
		double xPower = xController.calculate(currentPose.getX());
		double yPower = yController.calculate(currentPose.getY());
		double rotPower = rotController.calculate(currentPose.getRotation().getRadians());

		drivetrain.drive(xPower, yPower, rotPower);
	}

	@Override
	public void end(boolean interrupted) {
		Alerter.getInstance().rumble();
	}

	@Override
	public boolean isFinished() {
		return earlyExit || xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
	}
}
