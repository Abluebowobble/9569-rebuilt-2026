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

    private final static double A = 4.0;
    private final static double D = OperatorConstants.DEADBAND;
    private final static double N = 0.02;
    private final static double I1 = Math.pow(A, -D) * (1.0 - D);

    public InputShaper(DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier, DoubleSupplier turnSupplier) {
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.turnSupplier = turnSupplier;
    }

    public double getShapedXInput() {
        return shape(leftXSupplier.getAsDouble());
    }

    public double getShapedYInput() {
        return shape(leftYSupplier.getAsDouble());
    }

    public double getShapedTurnInput() {
        return shape(turnSupplier.getAsDouble());

    }

    private double shape(double x) {
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

        // k(x) = (1 - n) * (i(x) / i(1)) + n * sign(x)
        double k = (1.0 - N) * (iX / I1) + N * sign;

        return k;
    }
}