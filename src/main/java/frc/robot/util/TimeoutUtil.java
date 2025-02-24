package frc.robot.util;

public class TimeoutUtil {
    public static void setTimeout(Runnable r, long millis) {
        new Thread(() -> {
            try {
                Thread.sleep(millis);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            r.run();
        });
    }
}
