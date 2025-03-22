package frc.robot.event;

import java.util.ArrayList;
import java.util.function.Consumer;

public sealed class Event<T> permits VoidEvent {
    private final ArrayList<Consumer<T>> callbacks = new ArrayList<>();

    Event() { }

    @SafeVarargs
    Event(Event<T>... triggers) {
        for (Event<T> trigger : triggers) trigger.register(this::emit);
    }

    @SafeVarargs
    Event(EventTrigger<?, T>... triggers) {
        for (EventTrigger<?, T> trigger : triggers) trigger.register(this);
    }

    public void register(Consumer<T> callback) {
        callbacks.add(callback);
    }

    public void emit(T data) {
        for (Consumer<T> callback : callbacks) {
            callback.accept(data);
        }
    }
}
