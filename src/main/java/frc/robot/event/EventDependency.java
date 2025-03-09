package frc.robot.event;

import java.util.function.Function;

public class EventDependency<T, K> {
    final Event<K> other;
    final Function<K, T> transform;

    public EventDependency(Event<K> other, Function<K, T> transform) {
        this.other = other;
        this.transform = transform;
    }
}
