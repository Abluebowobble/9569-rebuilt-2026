// the states dont work for this 

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.Bank;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;

import frc.robot.Commands.GeneralRobotCommands.HoodState;
import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    // position variables, between 0-1
    public static final double kMinPosition = 0.05;
    public static final double kMaxPosition = 0.804;
    public static final double kStartingPosition = 0.4;

    // tolerance
    private static final double kPositionTolerance = 0.01;

    // sets current position and setpoint
    private double targetPosition = kStartingPosition;

    private HoodState hoodState = HoodState.IDLE;
    private final ServoHub servoHub = new ServoHub(HardwareMap.SERVO_HUB);
    private final ServoHubConfig servoConfig = new ServoHubConfig();
    private final ServoChannelConfig channelConfig = new ServoChannelConfig(ChannelId.kChannelId0);

    private ServoChannel leftServo;
    private ServoChannel rightServo;

    public HoodSubsystem() {
        channelConfig.pulseRange(1000, 1500, 2000)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kDoNotSupplyPower);
        servoConfig.apply(ChannelId.kChannelId0, channelConfig);
        servoConfig.apply(ChannelId.kChannelId5, channelConfig);

        servoHub.configure(servoConfig, ResetMode.kResetSafeParameters);
        servoHub.setBankPulsePeriod(Bank.kBank0_2, 4000);
        servoHub.setBankPulsePeriod(Bank.kBank3_5, 5000);

        rightServo = servoHub.getServoChannel(ChannelId.kChannelId0);
        leftServo = servoHub.getServoChannel(ChannelId.kChannelId5);

        leftServo.setPowered(true);
        leftServo.setEnabled(true);

        rightServo.setPowered(true);
        rightServo.setEnabled(true);

        // prep hood at maximal position
        setPosition(kStartingPosition);

        setDefaultCommand(idle());
    }

    /**
     * Expects a position between 0.0 and 1.0, sets position to given percentage
     * position
     */
    public void setPosition(DoubleSupplier position) {
        setPosition(position.getAsDouble());
    }

    public void setState(HoodState hoodState) {
        this.hoodState = hoodState;
    }

    public HoodState getState() {
        return hoodState;
    }

    /**
     * Expects a position between 0.0 and 1.0, sets position to given percentage
     * position
     */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);

        double pulseWidth = clampedPosition * 1000 + 1000;
        int pulseWidthInt = (int) Math.round(pulseWidth);

        leftServo.setPulseWidth(pulseWidthInt);
        rightServo.setPulseWidth(pulseWidthInt);

        targetPosition = clampedPosition;
    }

    /**
     * Expects a position between 0.0 and 1.0, sets the position to given value and
     * ends when position is within tolerance, for testing
     */
    public Command setCommand(DoubleSupplier position) {
        return run(() -> setPosition(position));
    }

    /**
     * Expects a position between 0.0 and 1.0, sets the position to given value and
     * ends when position is within tolerance
     */
    public Command setCommand(double position) {
        return run(() -> setPosition(position));
    }

    public Command feedFromNeutralCommand() {
        return setCommand(kMaxPosition);
    }

    @Override
    public Command idle() {
        return run(() -> setPosition(kMaxPosition / 2));
    }

    private double difference = 0;

    // public double progress() {

    // if (isPositionWithinTolerance()) {
    // difference = 0;
    // return 1.0;
    // }

    // double servoPosition = leftServo.getPosition();

    // if (difference == 0) {
    // difference = Math.abs(targetPosition - servoPosition);
    // }

    // double curDiff = servoPosition - targetPosition;
    // double changeInDiff = difference - curDiff;
    // double percentage = (changeInDiff / difference);
    // return percentage;
    // }

    @Override
    public void periodic() {
        // uncomment below to update hood based on smartdashboard
        // updateSpeedWithSmartDashboard();
        SmartDashboard.putData(this);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hood");

        builder.addDoubleProperty("Hood Target Position", () -> targetPosition, null);
    }

    public void updateSpeedWithSmartDashboard() {
        // setPosition(SmartDashboard.getNumber("Set Target Position", targetPosition));
    }
}
