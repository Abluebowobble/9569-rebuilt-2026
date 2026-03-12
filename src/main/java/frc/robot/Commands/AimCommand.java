// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.GeometryUtil;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LandMarks;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimCommand extends Command {
    private static final double kPoseEdgeMarginMeters = 0.1;
    private final SwerveSubsystem swerve;
    private boolean poseWarningIssued = false;
    private double rotationalVelocity;
    private DoubleSupplier leftYSupplier;
    private DoubleSupplier leftXSupplier;
    private Command operatorSwerveDefaulCommand;

    private Time lastUpdateTime = Seconds.of(Timer.getFPGATimestamp());

    /** Creates a new AimAndDriveCommand. */
    public AimCommand(SwerveSubsystem s, DoubleSupplier leftYSupplier, DoubleSupplier leftXSupplier,
            Command operatorSwerveDefaulCommand) {
        this.swerve = s;
        this.leftYSupplier = leftYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.operatorSwerveDefaulCommand = operatorSwerveDefaulCommand;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    private boolean currentPoseIsValid() {
        Pose2d pose = swerve.getPose();

        if (pose == null) {
            return false;
        }

        final Translation2d translation = pose.getTranslation();
        final double x = translation.getX();
        final double y = translation.getY();

        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return false;
        }

        return x >= -kPoseEdgeMarginMeters
                && x <= LandMarks.fieldLength + kPoseEdgeMarginMeters
                && y >= -kPoseEdgeMarginMeters
                && y <= LandMarks.fieldWidth + kPoseEdgeMarginMeters;
    }

    /**
     * sketchy asf would not trust, needs limit on angular velocity and linear
     * velocity
     */
    public double rotation() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        // find new rotational velocity
        rotationalVelocity = swerve.getSwerveDrive().getSwerveController()
                .headingCalculate(swerve.getHeading().getRadians(), swerve.getTargetHeadingInFieldFrame().getRadians());

        // convert into controller readings
        double velocityAsPercent = rotationalVelocity / swerve.getSwerveDrive().getMaximumChassisVelocity();
        double percentageChanged = velocityAsPercent * elapsedTime.magnitude();

        return percentageChanged;
    }

    private boolean isPoseValid(Pose2d pose) {
        if (pose == null) {
            return false;
        }
        
        final Translation2d translation = pose.getTranslation();
        final double x = translation.getX();
        final double y = translation.getY();
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return false;
        }

        return x >= -kPoseEdgeMarginMeters
                && x <= LandMarks.fieldLength + kPoseEdgeMarginMeters
                && y >= -kPoseEdgeMarginMeters
                && y <= LandMarks.fieldWidth + kPoseEdgeMarginMeters;
    }

    private boolean ensurePoseValidWithWarning() {
        final boolean valid = currentPoseIsValid();
        if (!valid) {
            if (!poseWarningIssued) {
                DriverStation.reportWarning("Auto aim blocked: robot pose outside field bounds", false);
                poseWarningIssued = true;
            }
        } else {
            poseWarningIssued = false;
        }
        return valid;
    }

    @Override
    public void initialize() {
        poseWarningIssued = false;
        ensurePoseValidWithWarning();

        // create swerve input stream for aim command
        SwerveInputStream driveAutoHeadingAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                leftYSupplier,
                leftXSupplier)
                .withControllerRotationAxis(this::rotation)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);
        Command driveFieldOrientedAutoHeadingAnglularVelocity = swerve
                .driveFieldOriented(driveAutoHeadingAngularVelocity);
        // override default command
        swerve.setDefaultCommand(driveFieldOrientedAutoHeadingAnglularVelocity);
    }

    @Override
    public boolean isFinished() {
        return !ensurePoseValidWithWarning();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setDefaultCommand(operatorSwerveDefaulCommand);
    }
}
