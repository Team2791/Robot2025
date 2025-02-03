package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.PIDConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoManager {
	final Drivetrain drivetrain;
	final PIDController orthoctl;
	final PIDController turnctl;

	final AutoFactory factory;

	public AutoManager(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		this.orthoctl = new PIDController(
			PIDConstants.Autos.kOrthoP,
			PIDConstants.Autos.kOrthoI,
			PIDConstants.Autos.kOrthoD
		);
		this.turnctl = new PIDController(
			PIDConstants.Autos.kTurnP,
			PIDConstants.Autos.kTurnI,
			PIDConstants.Autos.kTurnD
		);

		this.orthoctl.setTolerance(PIDConstants.Autos.kOrthoTolerance);
		this.turnctl.setTolerance(PIDConstants.Autos.kTurnTolerance);

		this.factory = new AutoFactory(
			drivetrain::getPose,
			drivetrain::resetOdometry,
			this::follow,
			true,
			drivetrain
		);
	}

	public void follow(SwerveSample traj) {
		Pose2d pose = drivetrain.getPose();
		ChassisSpeeds speeds = new ChassisSpeeds(
			traj.vy + orthoctl.calculate(pose.getX(), traj.x),
			traj.vx + orthoctl.calculate(pose.getY(), traj.y),
			traj.omega + turnctl.calculate(pose.getRotation().getRadians(), traj.heading)
		);

		drivetrain.drive(speeds);
	}

	public AutoRoutine orthoPidTuning() {
		AutoRoutine routine = factory.newRoutine("PID Tuning");
		AutoTrajectory traj = routine.trajectory("forward");
		Command seq = Commands.sequence(traj.resetOdometry(), traj.cmd());

		routine.active().onTrue(seq);

		return routine;
	}
}
