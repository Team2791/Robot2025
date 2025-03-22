package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.event.EventRegistry;
import frc.robot.util.Elastic.Notification.NotificationLevel;

import java.util.ArrayList;
import java.util.HashSet;

public class Alerter {
    private static Alerter instance;

    CommandXboxController driverctl;
    CommandXboxController operctl;

    ArrayList<SparkBase> sparks = new ArrayList<>();
    ArrayList<String> sparkNames = new ArrayList<>();
    HashSet<Integer> alertedSparks = new HashSet<>();

    AHRS gyro;
    boolean gyroAlerted;

    Notifier vibrateStop = new Notifier(this::still);

    private Alerter() {
        vibrateStop.setName("VibrateStop");
        EventRegistry.periodic.register(this::update);
    }

    public static Alerter getInstance() {
        if (instance == null) {
            instance = new Alerter();
        }

        return instance;
    }

    private static String makeHumanReadable(REVLibError error) {
        return switch (error) {
            case kOk -> "Everything is fine";
            case kError -> "General error";
            case kTimeout -> "Spark took too long to respond";
            case kNotImplemented -> "Function not implemented";
            case kHALError -> "Hardware abstraction layer error";
            case kCantFindFirmware -> "No firmware found on Spark";
            case kFirmwareTooOld -> "Firmware version is too old to be used with this library";
            case kFirmwareTooNew -> "Firmware version is too new to be used with this library";
            case kParamInvalidID -> "Invalid parameter ID";
            case kParamMismatchType -> "Parameter type mismatch";
            case kParamAccessMode -> "Parameter access mode mismatch";
            case kParamInvalid -> "Invalid parameter";
            case kParamNotImplementedDeprecated -> "Parameter not implemented or deprecated";
            case kFollowConfigMismatch -> "Follower configuration mismatch";
            case kInvalid -> "Invalid Spark configuration";
            case kSetpointOutOfRange -> "Motor setpoint out of range";
            case kUnknown -> "Unknown error";
            case kCANDisconnected -> "CAN bus was disconnected";
            case kDuplicateCANId -> "Duplicate CAN ID detected on bus";
            case kInvalidCANId -> "Spark has invalid can ID";
            case kSparkMaxDataPortAlreadyConfiguredDifferently -> "SparkMax data port already configured differently";
            case kSparkFlexBrushedWithoutDock -> "SparkFlex brushed motor without dock detected";
            case kInvalidBrushlessEncoderConfiguration -> "Invalid brushless encoder configuration";
            case kFeedbackSensorIncompatibleWithDataPortConfig -> "Sensor not compatible with data port configuration";
            case kParamInvalidChannel -> "Invalid parameter channel";
            case kParamInvalidValue -> "Invalid parameter value";
            case kCannotPersistParametersWhileEnabled -> "Cannot persist parameters while Spark is enabled";
        };
    }

    public void provideControllers(CommandXboxController driverctl, CommandXboxController operctl) {
        assert this.driverctl == null && this.operctl == null : "Controllers already provided";

        this.driverctl = driverctl;
        this.operctl = operctl;
    }

    public void rumble() {
        assert driverctl != null && operctl != null : "Controllers not provided";

        // no rumble in auto
        if (DriverStation.isAutonomous()) return;

        this.operctl.setRumble(RumbleType.kBothRumble, 1);
        this.driverctl.setRumble(RumbleType.kLeftRumble, 1);

        vibrateStop.stop();
        vibrateStop.startSingle(0.5);
    }

    private void still() {
        this.operctl.setRumble(RumbleType.kBothRumble, 0);
        this.driverctl.setRumble(RumbleType.kLeftRumble, 0);
    }

    public void registerSpark(String name, SparkBase spark) {
        sparks.add(spark);
        sparkNames.add(name);
    }

    public void registerGyro(AHRS gyro) {
        this.gyro = gyro;
        this.gyroAlerted = false;
    }

    public void update() {
        for (int i = 0; i < sparks.size(); i++) {
            SparkBase spark = sparks.get(i);
            String name = sparkNames.get(i);
            int can = spark.getDeviceId();

            if (!alertedSparks.contains(can) && spark.getLastError() != REVLibError.kOk) {
                Elastic.sendNotification(
                    new Elastic.Notification(
                        NotificationLevel.ERROR,
                        "Spark has failed",
                        name + "(CanId " + can + ") has failed with error: " + makeHumanReadable(spark.getLastError())
                    )
                );

                alertedSparks.add(can);
            }
        }

        if (!gyro.isConnected() && !gyroAlerted) {
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.WARNING,
                    "Gyro has disconnected",
                    "Using odometry as fallback - expect field-centric inaccuracies"
                )
            );
            gyroAlerted = true;
        } else if (gyro.isConnected() && gyroAlerted) {
            Elastic.sendNotification(
                new Elastic.Notification(
                    NotificationLevel.INFO,
                    "Gyro reconnected",
                    "No longer using odometry as fallback"
                )
            );
            gyroAlerted = false;
        }
    }
}
