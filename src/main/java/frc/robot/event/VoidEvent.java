package frc.robot.event;

public non-sealed class VoidEvent extends Event<Void> {
    public VoidEvent() { }

    public void emit() {
        super.emit(null);
    }

    public void register(Runnable callback) {
        super.register(v -> callback.run());
    }
}
