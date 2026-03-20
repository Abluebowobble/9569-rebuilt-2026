package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    // position variables
    public static final double kMinPosition = 0;
    public static final double kMaxPosition = 0.804;
    public static final double kStartingPosition = kMaxPosition / 2;

    // tolerance
    private static final double kPositionTolerance = 0.01;

    // sets current position and setpoint
    private double targetPosition = kStartingPosition;

    // servos
    private final Servo leftServo;
    private final Servo rightServo;

    public HoodSubsystem() {
        leftServo = new Servo(HardwareMap.ACTUATOR_LEFT);
        rightServo = new Servo(HardwareMap.ACTUATOR_RIGHT);

        // tune
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        // // adds value for SmartDashboard update function
        // SmartDashboard.putNumber("Set Target Position", 0.5);

        // prep hood at maximal position
        setPosition(kStartingPosition);
    }

    /**
     * Expects a position between 0.0 and 1.0, sets position to given percentage
     * position
     */
    public void setPosition(DoubleSupplier position) {
        setPosition(position.getAsDouble());
    }

    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);

        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);

        targetPosition = clampedPosition;
    }

    /**
     * Expects a position between 0.0 and 1.0, sets the position to given value and
     * ends when position is within tolerance, for testing
     */
    public Command setCommand(DoubleSupplier position) {
        return runOnce(() -> setPosition(position))
                .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    /**
     * Expects a position between 0.0 and 1.0, sets the position to given value and
     * ends when position is within tolerance
     */
    public Command setCommand(double position) {
        return runOnce(() -> setPosition(position))
                .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public Command feedFromNeutralCommand() {
        return setCommand(kMaxPosition);
    }

    public double progress() {
        if (isPositionWithinTolerance()) {
            return 1.0;
        }

        double servoPosition = leftServo.getPosition();

        if (servoPosition < targetPosition) {
            if (targetPosition <= 1e-9) {
                return 1.0;
            }
            return MathUtil.clamp(servoPosition / targetPosition, 0.0, 1.0);
        } else {
            double remainingRange = 1.0 - targetPosition;
            if (remainingRange <= 1e-9) {
                return 1.0;
            }
            return MathUtil.clamp((1.0 - servoPosition) / remainingRange, 0.0, 1.0);
        }
    }
 
    /** checks if current position is within given tolerance */
    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, leftServo.getPosition(), kPositionTolerance);
    }

    @Override
    public void periodic() {
        // uncomment below to update hood based on smartdashboard
        // updateSpeedWithSmartDashboard();

        SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hood");

        builder.addDoubleProperty("Hood Current Position Left", () -> leftServo.getPosition(), null);
        builder.addDoubleProperty("Hood Current Position Right", () -> rightServo.getPosition(), null);
        builder.addDoubleProperty("Hood Target Position", () -> targetPosition, null);
    }

    public void updateSpeedWithSmartDashboard() {
        // setPosition(SmartDashboard.getNumber("Set Target Position", targetPosition));
    }
}
