package bobot.utilities;

public class SlewController {

    private double slewRate, previousValue = 0;

    public SlewController(double slewRate) {
        this.slewRate = slewRate;
    }

    public void setSlewRate(double slewRate) {
        this.slewRate = slewRate;
    }

    public double calculate(double currentValue) {
        double output = currentValue;

        if (slewRate < Math.abs(currentValue - previousValue)) {
            if (previousValue < currentValue) output = previousValue - slewRate;
            else if (previousValue > currentValue) output = previousValue + slewRate;
        }

        previousValue = output;
        return output;
    }
}
