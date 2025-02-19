package frc.robot.constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class SparkConfigConstants {
    public static final class Elevator {
        public static final SparkFlexConfig kLeader;
        public static final SparkFlexConfig kFollower;

        static {
            kLeader = new SparkFlexConfig();
            kFollower = new SparkFlexConfig();

            // current limits
            kLeader.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);

            // position and velocity factors
            kLeader.encoder.positionConversionFactor(ElevatorConstants.Encoder.kPositionFactor);
            kFollower.encoder.positionConversionFactor(ElevatorConstants.Encoder.kPositionFactor);
            kLeader.encoder.velocityConversionFactor(ElevatorConstants.Encoder.kVelocityFactor);
            kFollower.encoder.velocityConversionFactor(ElevatorConstants.Encoder.kVelocityFactor);

            // pid constants
            kLeader.closedLoop.pidf(
                PIDConstants.Elevator.kP,
                PIDConstants.Elevator.kI,
                PIDConstants.Elevator.kD,
                PIDConstants.Elevator.kF
            );

            kLeader.closedLoop.outputRange(
                PIDConstants.Elevator.kMin,
                PIDConstants.Elevator.kMax
            );

            kFollower.closedLoop.pidf(
                PIDConstants.Elevator.kP,
                PIDConstants.Elevator.kI,
                PIDConstants.Elevator.kD,
                PIDConstants.Elevator.kF
            );

            kFollower.closedLoop.outputRange(
                PIDConstants.Elevator.kMin,
                PIDConstants.Elevator.kMax
            );

            // misc
            kLeader.idleMode(ElevatorConstants.Motor.kIdleMode);
            kFollower.idleMode(ElevatorConstants.Motor.kIdleMode);
            kFollower.follow(IOConstants.Elevator.kLeader, ElevatorConstants.Motor.kInvertFollower);
        }
    }

    public static final ResetMode kResetMode = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersistMode = PersistMode.kPersistParameters;
}
