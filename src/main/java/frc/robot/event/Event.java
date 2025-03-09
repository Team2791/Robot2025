package frc.robot.event;

public abstract class Event<T> {
    public Event() { }

    public EventDependency<T, ?> runAfter() { return null; }
}
