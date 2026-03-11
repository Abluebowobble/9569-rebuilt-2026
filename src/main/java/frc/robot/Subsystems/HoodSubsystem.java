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

    private double currentPosition = 0.5;
    private double targetPosition = 0.5;

    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);

    // servos
    private final Servo leftServo;
    private final Servo rightServo;

    // time tracker
    private Time lastUpdateTime = Seconds.of(0);


    public HoodSubsystem() {
        leftServo = new Servo(HardwareMap.ACTUATOR_LEFT);
        rightServo = new Servo(HardwareMap.ACTUATOR_RIGHT);

        // tune
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        setPosition(currentPosition);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);
        targetPosition = clampedPosition;

        SmartDashboard.putData(this);

    }

    /** Expects a position between 0.0 and 1.0 */
    public Command setCommand(double position) {
        return runOnce(() -> setPosition(position))
                .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
                ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
                : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
        SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
                null);
        sendableBuilder.addDoubleProperty("Current Position", () -> currentPosition, null);
        sendableBuilder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
        
    }
}
