package frc.robot.event;

import java.util.function.Consumer;

public class Key<T, E extends Event<T>> {
    final Class<E> key;
    final Consumer<T> listener;

    Key(Class<E> key, Consumer<T> listener) {
        this.key = key;
        this.listener = listener;
    }
}
