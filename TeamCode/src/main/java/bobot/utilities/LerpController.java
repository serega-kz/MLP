package bobot.utilities;

public class LerpController {

    private double rate, tolerance;

    private double lastTimeStamp = 0, duration = 0;
    private double startPosition = 0, endPosition = 0;

    public LerpController(double rate, double tolerance) {
        this.rate = rate;
        this.tolerance = tolerance;
    }

    public void setRateAndTolerance(double rate, double tolerance) {
        this.rate = rate;
        this.tolerance = tolerance;
    }

    public void setStartPosition(double startPosition) {
        this.startPosition = startPosition;
    }

    public void reset() {
        startPosition = 0;
        endPosition = 0;
        duration = 0;
    }

    public double getProgress() {
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        double progress = (currentTimeStamp - lastTimeStamp) / duration;

        if (duration == 0 || progress >= 1) {
            return 1;
        }

        return progress;
    }

    public double getEndPosition() {
        return endPosition;
    }

    public void setEndPosition(double endPosition) {
        if (getProgress() < 1) return;

        this.startPosition = this.endPosition;
        this.endPosition = endPosition;

        double delta = Math.abs(endPosition - startPosition);
        duration = delta <= tolerance ? 0 : delta / rate;

        lastTimeStamp = (double) System.nanoTime() / 1E9;
    }

    private double easeInOut(double x) {
        return Math.pow(Math.sin(Math.PI / 2 * x), 2);
    }

    private double lerp(double start, double end, double progress) {
        return start + (end - start) * progress;
    }

    public double calculate() {
        double progress = getProgress();
        if (progress >= 1) {
            return endPosition;
        }

        return lerp(startPosition, endPosition, easeInOut(progress));
    }
}
