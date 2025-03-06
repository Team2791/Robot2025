package frc.robot.event;

import java.util.HashMap;
import java.util.HashSet;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;
import java.util.function.Function;

@SuppressWarnings("rawtypes")
public class Emitter {
    public static class Dependency<T, K> {
        final Event<K> other;
        final Function<K, T> transform;

        public Dependency(Event<K> other, Function<K, T> transform) {
            this.other = other;
            this.transform = transform;
        }
    }

    public static class Key<T, E extends Event<T>> {
        final Class<E> key;
        final Consumer<T> listener;

        Key(Class<E> key, Consumer<T> listener) {
            this.key = key;
            this.listener = listener;
        }
    }

    public static abstract class Event<T> {
        public Event() { }

        public Dependency<T, ?> runAfter() { return null; }
    }

    // java complains about Class<? extends Event<?>> idk why
    private static final HashMap<Class<? extends Event>, HashSet<Consumer<?>>> listeners = new HashMap<>();

    // manage active jobs for paralleling/concurrency
    private static final ExecutorService executor = Executors.newCachedThreadPool();

    @SuppressWarnings("unchecked")
    public static <T, E extends Event<T>> void emit(E event, T data) {
        HashSet<Consumer<?>> eventListeners = listeners.get(event.getClass());

        if (eventListeners != null) {
            for (Consumer<?> listener : eventListeners) {
                executor.execute(() -> ((Consumer<T>) listener).accept(data));
            }
        }
    }

    public static <E extends Event<Void>> void emit(E event) {
        emit(event, null);
    }

    @SuppressWarnings("unchecked")
    public static <T, E extends Event<T>> Key<T, E> on(E event, Consumer<T> listener) {
        if (!listeners.containsKey(event.getClass())) {
            Dependency<T, Object> dep = ((Dependency<T, Object>) event.runAfter());

            if (dep != null) {
                listeners.computeIfAbsent(dep.other.getClass(), k -> new HashSet<>())
                    .add(k -> Emitter.emit(event, dep.transform.apply(k)));
            }
        }

        listeners.computeIfAbsent(event.getClass(), k -> new HashSet<>()).add(listener);

        return new Key<>(event.getClass(), listener);
    }

    public static <E extends Event<Void>> Key<Void, E> on(E event, Runnable listener) {
        return on(event, _v -> listener.run());
    }

    public static <T, E extends Event<T>> void off(Key<T, E> key) {
        HashSet<Consumer<?>> eventListeners = listeners.get(key.key);

        if (eventListeners != null) {
            eventListeners.remove(key.listener);
        }
    }
}
