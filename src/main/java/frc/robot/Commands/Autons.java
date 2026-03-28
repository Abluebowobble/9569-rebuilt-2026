package frc.robot.Commands;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.SilverKnightsLib.SwerveController;
import frc.robot.LandMarks;
import frc.robot.Constants.BehaviourConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.WaypointConstants;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class Autons {

    private static final Rotation2d kForward = Rotation2d.fromDegrees(0);
    private static final Rotation2d kBackward = Rotation2d.fromDegrees(180);
    private static final Rotation2d kIntakeHeading = Rotation2d.fromDegrees(-90);

    private static final double kLooseTolerance = 0.2;
    private static final double kMedTolerance = 0.1;
    private static final double kTightTolerance = 0.05;

    public static Command test(GeneralRobotCommands generalRobotCommands) {
        boolean isBlue = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Blue)
                .orElse(true);

        Translation2d start = allianceRelative(WaypointConstants.BLUE_1_START, isBlue);
        Rotation2d startHeading = isBlue ? kForward : kBackward;

        generalRobotCommands.getSwerveSubsystem().resetOdometry(new Pose2d(start, startHeading));

        Translation2d runup = allianceRelative(WaypointConstants.BLUE_1_RUNUP, isBlue);
        Translation2d beginIntake = allianceRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue);
        Translation2d finishIntake = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue);
        Translation2d prepareBump = allianceRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue);
        Translation2d returnFromBump = allianceRelative(WaypointConstants.BLUE_1_RETURN, isBlue);
        Translation2d shootPose = allianceRelative(WaypointConstants.BLUE_1_SHOOT, isBlue);
        Translation2d prepareDepot = allianceRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue);
        Translation2d depotIntake = allianceRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue);

        Rotation2d forwardHeading = isBlue ? kForward : kBackward;
        Rotation2d backwardHeading = isBlue ? kBackward : kForward;
        Rotation2d intakeHeading = isBlue ? kIntakeHeading : Rotation2d.fromDegrees(90);

        return generalRobotCommands.driveToWayPoint(
                runup,
                kMedTolerance,
                SwerveConstants.MAX_SPEED,
                null);
    }

    public static Command testleftAuton(GeneralRobotCommands generalRobotCommands) {
        var alliance = DriverStation.getAlliance();
        boolean isBlue;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            isBlue = true;
        } else {
            isBlue = false;
        }
        Translation2d backup = allianceRelative(WaypointConstants.BLUE_1_BACKUP, isBlue);
        Translation2d runup = allianceRelative(WaypointConstants.BLUE_1_RUNUP, isBlue);
        Translation2d beginIntake = allianceRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue);
        Translation2d finishIntake = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue);
        Translation2d prepareBump = allianceRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue);
        Translation2d returnFromBump = allianceRelative(WaypointConstants.BLUE_1_RETURN, isBlue);
        Translation2d shootPose = allianceRelative(WaypointConstants.BLUE_1_SHOOT, isBlue);
        Translation2d prepareDepot = allianceRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue);
        Translation2d depotIntake = allianceRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue);

        Rotation2d forwardHeading = isBlue ? kForward : kBackward;
        Rotation2d backwardHeading = isBlue ? kBackward : kForward;
        Rotation2d intakeHeading = isBlue ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90);

        return new SequentialCommandGroup(
                // generalRobotCommands.driveToWayPoint(
                // backup,
                // kMedTolerance,
                // SwerveConstants.MAX_SPEED,
                // null),
                new InstantCommand(() -> {
                    Translation2d start = allianceRelative(WaypointConstants.BLUE_1_START, isBlue);
                    Rotation2d startHeading = isBlue ? kBackward : kForward;

                    generalRobotCommands.getSwerveSubsystem().resetOdometry(new Pose2d(start, startHeading));
                }),

                generalRobotCommands.driveToWayPoint(
                        backup,
                        kMedTolerance,
                        isBlue ? SwerveConstants.MAX_SPEED.times(-1) : SwerveConstants.MAX_SPEED,
                        null),

                // run over the bump toward the neutral zone
                generalRobotCommands.driveToWayPoint(
                        runup,
                        0.5,
                        isBlue ? SwerveConstants.MAX_SPEED : SwerveConstants.MAX_SPEED.times(-1),
                        null),

                // intake from neutral zone while moving through the path
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                generalRobotCommands.driveToWithAngle(
                                        beginIntake,
                                        0.5,
                                        forwardHeading),
                                generalRobotCommands.driveToWayPointWithAngle(
                                        finishIntake,
                                        1.5,
                                        intakeHeading,
                                        null,
                                        isBlue ? SwerveConstants.MAX_SPEED.div(4).times(-1)
                                                : SwerveConstants.MAX_SPEED.div(4)),
                                generalRobotCommands.driveTo(
                                        finishIntake,
                                        kTightTolerance)),
                        generalRobotCommands.intakeCommand()),

                // test drive only === DO NOT USE FOR COMP
                // generalRobotCommands.driveToWithAngle(
                // beginIntake,
                // 0.5,
                // forwardHeading),
                // generalRobotCommands.driveToWithAngle(
                // finishIntake,
                // kTightTolerance,
                // intakeHeading),
                // =========

                // move back toward own side
                generalRobotCommands.driveToWithAngle(
                        prepareBump,
                        kMedTolerance,
                        backwardHeading),

                generalRobotCommands.driveToWayPoint(
                        returnFromBump,
                        kLooseTolerance,
                        isBlue ? SwerveConstants.MAX_SPEED.times(-1) : SwerveConstants.MAX_SPEED,
                        null),

                // first shot == TEST ONLY DO NOT RUN DURING COMP
                // generalRobotCommands.driveToWithAngle(
                // shootPose,
                // kTightTolerance,
                // forwardHeading));
                // ========== TEST ONLY TEST ONLY TEST ONLY
                new ParallelDeadlineGroup(
                        generalRobotCommands.driveToWithAngle(
                                shootPose,
                                kTightTolerance,
                                forwardHeading),
                        generalRobotCommands.getShooterSubsystem()
                                .runCommand(BehaviourConstants.TEMP_SHOOTER_VELOCITY)),

                shootWhenReady(generalRobotCommands, 1000));

        // // drive to depot
        // generalRobotCommands.driveToWithAngle(
        // prepareDepot,
        // kTightTolerance,
        // backwardHeading),

        // new ParallelDeadlineGroup(
        // generalRobotCommands.driveToWithAngle(
        // depotIntake,
        // kTightTolerance,
        // backwardHeading),
        // generalRobotCommands.intakeCommand()),

        // // leave depot
        // generalRobotCommands.driveToWithAngle(
        // prepareDepot,
        // 0.1,
        // backwardHeading),

        // // spin up while returning to the shot
        // new ParallelDeadlineGroup(
        // generalRobotCommands.driveToWithAngle(
        // shootPose,
        // kTightTolerance,
        // forwardHeading),
        // generalRobotCommands.getShooterSubsystem()
        // .runCommand(BehaviourConstants.TEMP_SHOOTER_VELOCITY)),

        // shootWhenReady(generalRobotCommands, 100000));
    }

    // private static Translation2d allianceRelative(Translation2d bluePoint,
    // boolean isBlue) {
    // if (isBlue) {
    // return bluePoint;
    // }

    // return new Translation2d(
    // LandMarks.kFieldLength.in(Meter) - bluePoint.getX(),
    // LandMarks.kFieldWidth.in(Meter) - bluePoint.getY());
    // }

    public static Command testRightAuton(GeneralRobotCommands generalRobotCommands) {
        var alliance = DriverStation.getAlliance();
        boolean isBlue;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            isBlue = true;
        } else {
            isBlue = false;
        }
        Translation2d backup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_BACKUP, isBlue));
        Translation2d runup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_RUNUP, isBlue));
        Translation2d beginIntake = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue));
        Translation2d finishIntake = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue));
        Translation2d prepareBump = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue));
        Translation2d returnFromBump = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_RETURN, isBlue));
        Translation2d shootPose = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_SHOOT, isBlue));
        Translation2d prepareDepot = mirrorSide(
                allianceRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue));
        Translation2d depotIntake = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue));
        Rotation2d forwardHeading = isBlue ? kForward : kBackward;
        Rotation2d backwardHeading = isBlue ? kBackward : kForward;
        Rotation2d intakeHeading = isBlue ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90);

        return new SequentialCommandGroup(
                // generalRobotCommands.driveToWayPoint(
                // backup,
                // kMedTolerance,
                // SwerveConstants.MAX_SPEED,
                // null),
                new InstantCommand(() -> {
                    Translation2d start = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_START, isBlue));
                    Rotation2d startHeading = isBlue ? kBackward : kForward;

                    generalRobotCommands.getSwerveSubsystem().getSwerveDrive()
                            .resetOdometry(new Pose2d(start, startHeading));

                }),

                // run over the bump toward the neutral zone
                generalRobotCommands.driveToWayPoint(
                        backup,
                        kMedTolerance,
                        isBlue ? SwerveConstants.MAX_SPEED.times(-1) : SwerveConstants.MAX_SPEED,
                        null),

                // run over the bump toward the neutral zone
                generalRobotCommands.driveToWayPoint(
                        runup,
                        0.5,
                        isBlue ? SwerveConstants.MAX_SPEED : SwerveConstants.MAX_SPEED.times(-1),
                        null),

                // intake from neutral zone while moving through the path
                // new ParallelDeadlineGroup(
                // new SequentialCommandGroup(
                // generalRobotCommands.driveToWithAngle(
                // beginIntake,
                // 0.5,
                // forwardHeading),
                // generalRobotCommands.driveToWithAngle(
                // finishIntake,
                // kTightTolerance,
                // intakeHeading)),
                // generalRobotCommands.intakeCommand()),

                generalRobotCommands.driveToWithAngle(
                        beginIntake,
                        0.5,
                        forwardHeading),
                generalRobotCommands.driveToWithAngle(
                        finishIntake,
                        kTightTolerance,
                        intakeHeading),

                // move back toward own side
                generalRobotCommands.driveToWithAngle(
                        prepareBump,
                        kMedTolerance,
                        backwardHeading),

                generalRobotCommands.driveToWayPoint(
                        returnFromBump,
                        kLooseTolerance,
                        isBlue ? SwerveConstants.MAX_SPEED.times(-1) : SwerveConstants.MAX_SPEED,
                        null),

                // first shot
                generalRobotCommands.driveToWithAngle(
                        shootPose,
                        kTightTolerance,
                        forwardHeading));
        // new ParallelDeadlineGroup(
        // generalRobotCommands.driveToWithAngle(
        // shootPose,
        // kTightTolerance,
        // forwardHeading),
        // generalRobotCommands.getShooterSubsystem()
        // .runCommand(BehaviourConstants.TEMP_SHOOTER_VELOCITY)),

        // shootWhenReady(generalRobotCommands, 1000));

        // // drive to depot
        // generalRobotCommands.driveToWithAngle(
        // prepareDepot,
        // kTightTolerance,
        // backwardHeading),

        // new ParallelDeadlineGroup(
        // generalRobotCommands.driveToWithAngle(
        // depotIntake,
        // kTightTolerance,
        // backwardHeading),
        // generalRobotCommands.intakeCommand()),

        // // leave depot
        // generalRobotCommands.driveToWithAngle(
        // prepareDepot,
        // 0.1,
        // backwardHeading),

        // // spin up while returning to the shot
        // new ParallelDeadlineGroup(
        // generalRobotCommands.driveToWithAngle(
        // shootPose,
        // kTightTolerance,
        // forwardHeading),
        // generalRobotCommands.getShooterSubsystem()
        // .runCommand(BehaviourConstants.TEMP_SHOOTER_VELOCITY)),

        // shootWhenReady(generalRobotCommands, 100000));
    }

    private static Translation2d allianceRelative(Translation2d pose, boolean isBlue) {

        if (!isBlue) {
            double newX = LandMarks.kFieldLength.in(Meter) - pose.getX();
            double newY = LandMarks.kFieldWidth.in(Meter) - pose.getY();

            return new Translation2d(newX, newY);
        }

        return pose;
    }

    public static Translation2d mirrorSide(Translation2d point) {
        return new Translation2d(
                point.getX(),
                LandMarks.kFieldWidth.magnitude() - point.getY());
    }

    private static Command shootWhenReady(GeneralRobotCommands generalRobotCommands, double seconds) {
        return Commands.waitUntil(generalRobotCommands::isReadyToShoot)
                .andThen(
                        new ParallelDeadlineGroup(
                                Commands.waitSeconds(seconds),
                                generalRobotCommands.feedCommand(),
                                generalRobotCommands.getShooterSubsystem()
                                        .runCommand(BehaviourConstants.TEMP_SHOOTER_VELOCITY)));
    }

    public static Command testForwardAuton(GeneralRobotCommands generalRobotCommands) {
        boolean isBlue = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Blue)
                .orElse(true);

        Translation2d start = allianceRelative(WaypointConstants.BLUE_1_START, isBlue);
        Rotation2d startHeading = isBlue ? kForward : kBackward;

        Translation2d forwardPoint = new Translation2d(
                start.getX() + (isBlue ? -0.5 : 0.5),
                start.getY());

        return new SequentialCommandGroup(
                new InstantCommand(() -> generalRobotCommands.getSwerveSubsystem()
                        .resetOdometry(new Pose2d(start, startHeading))),

                generalRobotCommands.driveToWayPoint(
                        forwardPoint,
                        kTightTolerance,
                        SwerveConstants.MAX_SPEED,
                        null));
    }

    public static Command testRotateAuton(GeneralRobotCommands generalRobotCommands) {
        boolean isBlue = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Blue)
                .orElse(true);

        Translation2d start = allianceRelative(WaypointConstants.BLUE_1_START, isBlue);
        Rotation2d startHeading = isBlue ? kBackward : kForward;
        Rotation2d rotatedHeading = startHeading.plus(Rotation2d.fromDegrees(90));

        return new SequentialCommandGroup(
                new InstantCommand(() -> generalRobotCommands.getSwerveSubsystem()
                        .resetOdometry(new Pose2d(start, startHeading))),

                generalRobotCommands.driveToWithAngle(
                        start,
                        kTightTolerance,
                        rotatedHeading));
    }

}
