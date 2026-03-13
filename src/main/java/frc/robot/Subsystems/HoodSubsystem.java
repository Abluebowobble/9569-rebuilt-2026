package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

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
    // position variables, to tune
    private static final double kMinPosition = 0;
    private static final double kMaxPosition = 0;
    private static final double kPositionTolerance = 0;

    // sets current position and setpoint
    private static final double kStartingPosition = 0.5;
    private double targetPosition = 0.5;

    // servos
    private final Servo leftServo;
    private final Servo rightServo;
    
    public HoodSubsystem() {
        leftServo = new Servo(HardwareMap.ACTUATOR_LEFT);
        rightServo = new Servo(HardwareMap.ACTUATOR_RIGHT);

        // tune
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        // move hood to 50% on initialization
        setPosition(kStartingPosition);

        // adds value for SmartDashboard update function
        SmartDashboard.putNumber("Set Target Position", 0.5);
    }

    /**
     * Expects a position between 0.0 and 1.0, sets position to given percentage
     * position
     */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);

        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);

        targetPosition = clampedPosition;
    }

    /**
     * Expects a position between 0.0 and 1.0, sets the position to given value and
     * ends when position is within tolerance
     */
    public Command setCommand(double position) {
        return runOnce(() -> setPosition(position))
                .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    /** checks if current position is within given tolerance */
    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, leftServo.getPosition(), kPositionTolerance);
    }

    @Override
    public void periodic() {
        // uncomment below to update hood based on smartdashboard
        // updateSpeedWithSmartDashboard();

        // telemetry
        SmartDashboard.putNumber("Current Position", leftServo.getPosition());
        SmartDashboard.putNumber("Target Position", targetPosition);
    }

    public void updateSpeedWithSmartDashboard() {
        setPosition(SmartDashboard.getNumber("Set Target Position", targetPosition));
    }
}
