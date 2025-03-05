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
            kLeader.encoder.velocityConversionFactor(ElevatorConstants.Encoder.kVelocityFactor);

            // pid constants
            kLeader.closedLoop.pidf(
                ControlConstants.Elevator.kP,
                ControlConstants.Elevator.kI,
                ControlConstants.Elevator.kD,
                ControlConstants.Elevator.kF
            );

            kLeader.closedLoop.outputRange(
                ControlConstants.Elevator.kMin,
                ControlConstants.Elevator.kMax
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
                ControlConstants.Dispenser.kP,
                ControlConstants.Dispenser.kI,
                ControlConstants.Dispenser.kD,
                ControlConstants.Dispenser.kF
            );

            kLeader.closedLoop.outputRange(
                ControlConstants.Dispenser.kMin,
                ControlConstants.Dispenser.kMax
            );

            kFollower.closedLoop.pidf(
                ControlConstants.Dispenser.kP,
                ControlConstants.Dispenser.kI,
                ControlConstants.Dispenser.kD,
                ControlConstants.Dispenser.kF
            );

            kFollower.closedLoop.outputRange(
                ControlConstants.Dispenser.kMin,
                ControlConstants.Dispenser.kMax
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

    public static final class Roller {
        public static final SparkMaxConfig kLeft;
        public static final SparkMaxConfig kRight;

        static {
            kLeft = new SparkMaxConfig();
            kRight = new SparkMaxConfig();

            // current limits
            kLeft.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);
            kRight.smartCurrentLimit((int) MotorConstants.Neo.kCurrentLimit);

            // position and velocity factors
            kLeft.encoder.positionConversionFactor(IntakeConstants.Encoder.kPositionFactor);
            kRight.encoder.positionConversionFactor(IntakeConstants.Encoder.kPositionFactor);
            kLeft.encoder.velocityConversionFactor(IntakeConstants.Encoder.kVelocityFactor);
            kRight.encoder.velocityConversionFactor(IntakeConstants.Encoder.kVelocityFactor);

            // misc
            kLeft.idleMode(IntakeConstants.Motor.kIdleMode);
            kRight.idleMode(IntakeConstants.Motor.kIdleMode);

            // beam brake
            kLeft.limitSwitch.reverseLimitSwitchEnabled(true);
            kLeft.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        }
    }

    public static final ResetMode kResetMode = ResetMode.kResetSafeParameters;
    public static final PersistMode kPersistMode = PersistMode.kPersistParameters;
}
