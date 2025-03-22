package frc.robot.event;

import java.util.function.Function;

public class EventTrigger<T, K> {
    final Event<T> trigger;
    final Function<T, K> transform;

    public EventTrigger(Event<T> trigger, Function<T, K> transform) {
        this.trigger = trigger;
        this.transform = transform;
    }

    void register(Event<K> self) {
        trigger.register(data -> self.emit(transform.apply(data)));
    }
}
