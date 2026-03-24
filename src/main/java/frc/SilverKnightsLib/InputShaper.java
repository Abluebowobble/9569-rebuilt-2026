/**
 * thank you to lem for their expo drive equation :)
 * values can be adjusted here
 * https://www.desmos.com/calculator/umicbymbnl
 */

package frc.SilverKnightsLib;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.OperatorConstants;

public class InputShaper {

    private final DoubleSupplier leftXSupplier;
    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier turnSupplier;

    public InputShaper(DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier, DoubleSupplier turnSupplier) {
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.turnSupplier = turnSupplier;
    }

    public double getShapedXInput() {
        return shape(leftXSupplier.getAsDouble(), 4, 0.05, 0.02);
    }

    public double getShapedYInput() {
        return shape(leftYSupplier.getAsDouble(), 4, 0.05, 0.02);
    }

    public double getShapedTurnInput() {
        return shape(turnSupplier.getAsDouble(), 4, 0.05, 0.02);

    }

    private double shape(double x, double A, double D, double N) {
        double absX = Math.abs(x);

        // deadband
        if (absX < D) {
            return 0.0;
        }

        double sign = Math.signum(x);

        // g(x) = |x| - d
        double g = absX - D;

        // i(x) = a^(g(x)-1) * g(x) * sign(x)
        double iX = Math.pow(A, g - 1.0) * g * sign;

        double I1 = Math.pow(A, -D) * (1.0 - D);

        // k(x) = (1 - n) * (i(x) / i(1)) + n * sign(x)
        double k = (1.0 - N) * (iX / I1) + N * sign;

        return k;
    }
}