package frc.robot.event;

import edu.wpi.first.math.geometry.Pose2d;

public class EventRegistry {
    public static final VoidEvent periodic = new VoidEvent();

    public static final Event<Pose2d> poseReset = new Event<>();
    public static final Event<Pose2d> poseUpdate = new Event<>(poseReset);
}
