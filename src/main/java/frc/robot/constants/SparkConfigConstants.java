package frc.robot.constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    public static final class Dispenser {
        public static final SparkMaxConfig kLeader;
        public static final SparkMaxConfig kFollower;

        static {
            kLeader = new SparkMaxConfig();
            kFollower = new SparkMaxConfig();

            // current limits
            kLeader.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);
            kFollower.smartCurrentLimit((int) MotorConstants.NeoVortex.kCurrentLimit);

            // position and velocity factors
            kLeader.encoder.positionConversionFactor(DispenserConstants.Encoder.kPositionFactor);
            kFollower.encoder.positionConversionFactor(DispenserConstants.Encoder.kPositionFactor);
            kLeader.encoder.velocityConversionFactor(DispenserConstants.Encoder.kVelocityFactor);
            kFollower.encoder.velocityConversionFactor(DispenserConstants.Encoder.kVelocityFactor);

            // pid constants
            kLeader.closedLoop.pidf(
                PIDConstants.Dispenser.kP,
                PIDConstants.Dispenser.kI,
                PIDConstants.Dispenser.kD,
                PIDConstants.Dispenser.kF
            );

            kLeader.closedLoop.outputRange(
                PIDConstants.Dispenser.kMin,
                PIDConstants.Dispenser.kMax
            );

            kFollower.closedLoop.pidf(
                PIDConstants.Dispenser.kP,
                PIDConstants.Dispenser.kI,
                PIDConstants.Dispenser.kD,
                PIDConstants.Dispenser.kF
            );

            kFollower.closedLoop.outputRange(
                PIDConstants.Dispenser.kMin,
                PIDConstants.Dispenser.kMax
            );

            // misc
            kLeader.idleMode(DispenserConstants.Motor.kIdleMode);
            kFollower.idleMode(DispenserConstants.Motor.kIdleMode);
            kFollower.follow(IOConstants.Dispenser.kLeader, DispenserConstants.Motor.kInvertFollower);

            // beam brake
            kLeader.limitSwitch.reverseLimitSwitchEnabled(true);
            kLeader.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        }
    }

    public static final ResetMode kResetMode = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersistMode = PersistMode.kPersistParameters;
}
