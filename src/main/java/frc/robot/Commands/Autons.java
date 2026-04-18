package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LandMarks;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.WaypointConstants;

public class Autons {

        private static final Rotation2d kForward = Rotation2d.fromDegrees(0);
        private static final Rotation2d kBackward = Rotation2d.fromDegrees(180);
        private static final Rotation2d kIntakeHeading = Rotation2d.fromDegrees(-90);
        private static final Rotation2d kBumpHeadingBottomLeft = Rotation2d.fromDegrees(-135);
        private static final Rotation2d kBumpHeadingTopLeft = Rotation2d.fromDegrees(135);
        private static final Rotation2d kBumpHeadingTopRight = Rotation2d.fromDegrees(45);
        private static final Rotation2d kBumpHeadingBottomRight = Rotation2d.fromDegrees(-45);

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

        public static Command leftAuton(GeneralRobotCommands generalRobotCommands) {
                var alliance = DriverStation.getAlliance();
                boolean isBlue;
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                        isBlue = true;
                } else {
                        isBlue = false;
                }

                Translation2d backup = allianceRelative(WaypointConstants.BLUE_1_BACKUP, isBlue);
                Translation2d runup = allianceRelative(WaypointConstants.BLUE_1_RUNUP, isBlue);
                Translation2d frontTrench = allianceRelative(WaypointConstants.BLUE_1_FRONT_TRENCH, isBlue);
                Translation2d beginIntake = allianceRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue);
                Translation2d finishIntake = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue);
                Translation2d finishIntake2 = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE2, isBlue);
                Translation2d prepareBump = allianceRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue);
                Translation2d returnFromBump = allianceRelative(WaypointConstants.BLUE_1_RETURN, isBlue);
                Translation2d shootPose = allianceRelative(WaypointConstants.BLUE_1_SHOOT, isBlue);
                Translation2d prepareDepot = allianceRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue);
                Translation2d depotIntake = allianceRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue);
                Translation2d finalPos = allianceRelative(WaypointConstants.BLUE_1_FINAL, isBlue);

                Rotation2d forwardHeading = isBlue ? kForward : kBackward;
                Rotation2d backwardHeading = isBlue ? kBackward : kForward;
                Rotation2d intakeHeading = isBlue ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90);
                Rotation2d bumpHeading = isBlue ? kBumpHeadingBottomRight : kBumpHeadingTopLeft;

                final double SHOOT_DELAY = 4;

                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        Translation2d start = allianceRelative(WaypointConstants.BLUE_1_START, isBlue);

                                        generalRobotCommands.getSwerveSubsystem()
                                                        .resetOdometry(new Pose2d(start, bumpHeading));
                                }),

                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.5)
                                                                : SwerveConstants.MAX_SPEED.times(0.5).times(-1),
                                                null),

                                generalRobotCommands.driveToWayPoint(
                                                frontTrench,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                                                : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                                isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                                                : SwerveConstants.MAX_SPEED),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPoint(
                                                                                finishIntake,
                                                                                4.5,
                                                                                null,
                                                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                                                .times(0.2).times(-1)
                                                                                                : SwerveConstants.MAX_SPEED
                                                                                                                .times(0.2)),
                                                                generalRobotCommands.driveToWithAngle(
                                                                                prepareBump,
                                                                                kMedTolerance,
                                                                                bumpHeading)),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())),

                                generalRobotCommands.driveToWayPoint(
                                                returnFromBump,
                                                0.5,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.5).times(-1)
                                                                : SwerveConstants.MAX_SPEED.times(0.5),
                                                null),

                                generalRobotCommands.driveToWithAngle(
                                                returnFromBump,
                                                kTightTolerance,
                                                bumpHeading),

                                new ParallelDeadlineGroup(
                                                Commands.waitSeconds(0.5)
                                                                .andThen(generalRobotCommands.feedCommand())
                                                                .withTimeout(SHOOT_DELAY),
                                                generalRobotCommands.prepareShooter()),

                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.5)
                                                                : SwerveConstants.MAX_SPEED.times(0.5).times(-1),
                                                null),

                                generalRobotCommands.driveToWayPoint(
                                                frontTrench,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                                                : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                                isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                                                : SwerveConstants.MAX_SPEED),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPoint(
                                                                                finishIntake,
                                                                                4.5,
                                                                                null,
                                                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                                                .times(0.2).times(-1)
                                                                                                : SwerveConstants.MAX_SPEED
                                                                                                                .times(0.2))),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())),

                                generalRobotCommands.driveToWithAngle(
                                                prepareBump,
                                                kMedTolerance,
                                                bumpHeading));
        }

        public static Command rightAuton(GeneralRobotCommands generalRobotCommands) {
                var alliance = DriverStation.getAlliance();
                boolean isBlue;
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                        isBlue = true;
                } else {
                        isBlue = false;
                }

                Translation2d backup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_BACKUP, isBlue));
                Translation2d runup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_RUNUP, isBlue));
                Translation2d frontTrench = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_FRONT_TRENCH, isBlue));
                Translation2d beginIntake = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue));
                Translation2d finishIntake = mirrorSide(
                                allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue));
                Translation2d finishIntake2 = mirrorSide(
                                allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE2, isBlue));
                Translation2d prepareBump = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue));
                Translation2d returnFromBump = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_RETURN, isBlue));
                Translation2d shootPose = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_SHOOT, isBlue));
                Translation2d prepareDepot = mirrorSide(
                                allianceRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue));
                Translation2d depotIntake = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue));
                Translation2d finalPos = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_FINAL, isBlue));

                Rotation2d forwardHeading = isBlue ? kForward : kBackward;
                Rotation2d backwardHeading = isBlue ? kBackward : kForward;
                Rotation2d intakeHeading = isBlue ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90);
                Rotation2d bumpHeading = isBlue ? kBumpHeadingTopRight : kBumpHeadingBottomLeft;

                final double SHOOT_DELAY = 4;

                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        Translation2d start = mirrorSide(
                                                        allianceRelative(WaypointConstants.BLUE_1_START, isBlue));

                                        generalRobotCommands.getSwerveSubsystem()
                                                        .resetOdometry(new Pose2d(start, bumpHeading));
                                }),

                                // generalRobotCommands.driveToWayPoint(
                                // backup,
                                // kMedTolerance,
                                // isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                // : SwerveConstants.MAX_SPEED,
                                // null),

                                // run over the bump toward the neutral zone
                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.5)
                                                                : SwerveConstants.MAX_SPEED.times(0.5).times(-1),
                                                null),

                                // intake from neutral zone while moving through the path
                                // new ParallelDeadlineGroup(
                                // new SequentialCommandGroup(
                                // generalRobotCommands.driveToWithAngle(
                                // beginIntake,
                                // 0.5,
                                // forwardHeading),
                                // generalRobotCommands.driveToWayPointWithAngle(
                                // finishIntake,
                                // 1.5,
                                // intakeHeading,
                                // null,
                                // isBlue ? SwerveConstants.MAX_SPEED.div(4).times(-1)
                                // : SwerveConstants.MAX_SPEED.div(4)),
                                // generalRobotCommands.driveTo(
                                // finishIntake2,
                                // kTightTolerance)),
                                // generalRobotCommands.intakeCommand()),

                                // test drive only === DO NOT USE FOR COMP
                                generalRobotCommands.driveToWayPoint(frontTrench, 1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                                                : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                .times(-1)
                                                                : SwerveConstants.MAX_SPEED),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPoint(
                                                                                finishIntake,
                                                                                4.5,
                                                                                null,
                                                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                                                .times(0.2)
                                                                                                : SwerveConstants.MAX_SPEED
                                                                                                                .times(0.2)
                                                                                                                .times(-1)),
                                                                generalRobotCommands.driveToWithAngle(
                                                                                prepareBump,
                                                                                kMedTolerance,
                                                                                bumpHeading)),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())),
                                // make sure to face the right way
                                // generalRobotCommands.driveToWithAngle(
                                // generalRobotCommands.getSwerveSubsystem().getPose().getTranslation(),
                                // kMedTolerance,
                                // intakeHeading),
                                // drive through the middle, end point is large to mimic like a line basically

                                // generalRobotCommands.driveTo(
                                // finishIntake2,
                                // kTightTolerance),
                                // =========

                                // move back toward own side

                                generalRobotCommands.driveToWayPoint(
                                                returnFromBump,
                                                0.5,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.5).times(-1)
                                                                : SwerveConstants.MAX_SPEED.times(0.5),
                                                null),

                                generalRobotCommands.driveToWithAngle(
                                                returnFromBump,
                                                kTightTolerance,
                                                bumpHeading),

                                new ParallelDeadlineGroup(
                                                Commands.waitSeconds(0.5).andThen(
                                                                generalRobotCommands.feedCommand())
                                                                .withTimeout(SHOOT_DELAY),
                                                generalRobotCommands.prepareShooter()),
                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.5)
                                                                : SwerveConstants.MAX_SPEED.times(0.5).times(-1),
                                                null),

                                // intake from neutral zone while moving through the path
                                // new ParallelDeadlineGroup(
                                // new SequentialCommandGroup(
                                // generalRobotCommands.driveToWithAngle(
                                // beginIntake,
                                // 0.5,
                                // forwardHeading),
                                // generalRobotCommands.driveToWayPointWithAngle(
                                // finishIntake,
                                // 1.5,
                                // intakeHeading,
                                // null,
                                // isBlue ? SwerveConstants.MAX_SPEED.div(4).times(-1)
                                // : SwerveConstants.MAX_SPEED.div(4)),
                                // generalRobotCommands.driveTo(
                                // finishIntake2,
                                // kTightTolerance)),
                                // generalRobotCommands.intakeCommand()),

                                // test drive only === DO NOT USE FOR COMP
                                generalRobotCommands.driveToWayPoint(frontTrench, 1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                                                : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                .times(-1)
                                                                : SwerveConstants.MAX_SPEED),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPoint(
                                                                                finishIntake,
                                                                                4.5,
                                                                                null,
                                                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                                                .times(0.2)
                                                                                                : SwerveConstants.MAX_SPEED
                                                                                                                .times(0.2)
                                                                                                                .times(-1))),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())),

                                generalRobotCommands.driveToWithAngle(
                                                prepareBump,
                                                kMedTolerance,
                                                bumpHeading));
        }

        public static Command rightLockMiddle(GeneralRobotCommands generalRobotCommands) {
                var alliance = DriverStation.getAlliance();
                boolean isBlue;
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                        isBlue = true;
                } else {
                        isBlue = false;
                }
                Translation2d backup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_BACKUP, isBlue));
                Translation2d runup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_RUNUP, isBlue));
                Translation2d beginIntake = allianceRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue);
                Translation2d finishIntake = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue);
                Translation2d finishIntake2 = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE2, isBlue);
                Translation2d prepareBump = allianceRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue);
                Translation2d returnFromBump = allianceRelative(WaypointConstants.BLUE_1_RETURN, isBlue);
                Translation2d shootPose = allianceRelative(WaypointConstants.BLUE_1_SHOOT, isBlue);
                Translation2d prepareDepot = allianceRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue);
                Translation2d depotIntake = allianceRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue);
                Translation2d temp = mirrorSide(allianceRelative(WaypointConstants.TEMP, isBlue));

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
                                        Translation2d start = mirrorSide(
                                                        allianceRelative(WaypointConstants.BLUE_1_START, isBlue));
                                        Rotation2d startHeading = isBlue ? kForward : kBackward;

                                        generalRobotCommands.getSwerveSubsystem().getSwerveDrive()
                                                        .resetOdometry(new Pose2d(start, startHeading));

                                }),

                                // run over the bump toward the neutral zone
                                generalRobotCommands.driveToWayPoint(
                                                backup,
                                                kMedTolerance,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                                                : SwerveConstants.MAX_SPEED,
                                                null),

                                // run over the bump toward the neutral zone
                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                0.5,
                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                : SwerveConstants.MAX_SPEED.times(-1),
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

                                // intake middle
                                generalRobotCommands.driveToWithAngle(
                                                temp,
                                                kTightTolerance,
                                                forwardHeading),
                                // generalRobotCommands.driveToWithAngle(
                                // finishIntake,
                                // 4.5,
                                // intakeHeading),
                                // generalRobotCommands.driveToWithAngle(
                                // finishIntake2,
                                // kTightTolerance,
                                // intakeHeading),

                                new InstantCommand(() -> generalRobotCommands.getSwerveSubsystem().lockPose())
                                                .repeatedly());

                // // move back toward own side
                // generalRobotCommands.driveToWithAngle(
                // prepareBump,
                // kMedTolerance,
                // backwardHeading),

                // generalRobotCommands.driveToWayPoint(
                // returnFromBump,
                // kLooseTolerance,
                // isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                // : SwerveConstants.MAX_SPEED,
                // null),

                // // first shot
                // generalRobotCommands.driveToWithAngle(
                // shootPose,
                // kTightTolerance,
                // forwardHeading));
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

        public static Translation2d allianceRelative(Translation2d pose, boolean isBlue) {

                if (!isBlue) {
                        double newX = LandMarks.kFieldLength.in(Meter) - pose.getX();
                        double newY = LandMarks.kFieldWidth.in(Meter) - pose.getY();

                        return new Translation2d(newX, newY);
                }

                return pose;
        }

        public static Command sideAuton(GeneralRobotCommands generalRobotCommands, boolean isLeft) {
                var alliance = DriverStation.getAlliance();
                boolean isBlue;
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                        isBlue = true;
                } else {
                        isBlue = false;
                }

                Translation2d backup = sideRelative(WaypointConstants.BLUE_1_BACKUP, isBlue, isLeft);
                Translation2d runup = sideRelative(WaypointConstants.BLUE_1_RUNUP, isBlue, isLeft);
                Translation2d frontTrench = sideRelative(WaypointConstants.BLUE_1_FRONT_TRENCH, isBlue, isLeft);
                Translation2d beginIntake = sideRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue, isLeft);
                Translation2d beginIntake2 = sideRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE_2, isBlue, isLeft);
                Translation2d finishIntake = sideRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue, isLeft);
                Translation2d finishIntake2 = sideRelative(WaypointConstants.BLUE_1_FINISH_INTAKE_2, isBlue, isLeft);
                Translation2d prepareBump = sideRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue, isLeft);
                Translation2d returnFromBump = sideRelative(WaypointConstants.BLUE_1_RETURN, isBlue, isLeft);
                Translation2d shootPose = sideRelative(WaypointConstants.BLUE_1_SHOOT, isBlue, isLeft);
                Translation2d prepareDepot = sideRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue,
                                isLeft);
                Translation2d depotIntake = sideRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue, isLeft);
                Translation2d finalPos = sideRelative(WaypointConstants.BLUE_1_FINAL, isBlue, isLeft);

                Rotation2d forwardHeading = isBlue ? kForward : kBackward;
                Rotation2d backwardHeading = isBlue ? kBackward : kForward;

                Rotation2d intakeHeading = isLeft
                                ? (isBlue ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90))
                                : (isBlue ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90));

                Rotation2d bumpHeading = isLeft
                                ? (isBlue ? kBumpHeadingBottomRight : kBumpHeadingTopLeft)
                                : (isBlue ? kBumpHeadingTopRight : kBumpHeadingBottomLeft);

                double percentageIntakeSpeed = 0.4;
                double percentageBumpSpeed = 0.6;

                LinearVelocity intakeVelocity = isLeft
                                ? (isBlue ? SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed).times(-1)
                                                : SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed))
                                : (isBlue ? SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed)
                                                : SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed).times(-1));

                final double SHOOT_DELAY = 4.5;

                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        Translation2d start = sideRelative(WaypointConstants.BLUE_1_START, isBlue,
                                                        isLeft);

                                        generalRobotCommands.getSwerveSubsystem()
                                                        .resetOdometry(new Pose2d(start, bumpHeading));
                                }),

                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                : SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                                .times(-1),
                                                null),

                                // generalRobotCommands.driveToWayPoint(
                                // frontTrench,
                                // 1,
                                // isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                // : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                // isLeft
                                // ? (isBlue ? SwerveConstants.MAX_SPEED
                                // : SwerveConstants.MAX_SPEED.times(-1))
                                // : (isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                // : SwerveConstants.MAX_SPEED)),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPointWithAngle(
                                                                                finishIntake,
                                                                                4.5,
                                                                                intakeHeading,
                                                                                null,
                                                                                intakeVelocity),
                                                                generalRobotCommands.driveToWithAngle(
                                                                                prepareBump,
                                                                                kMedTolerance,
                                                                                bumpHeading)),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())),

                                generalRobotCommands.driveToWayPoint(
                                                returnFromBump,
                                                0.5,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(percentageBumpSpeed).times(-1)
                                                                : SwerveConstants.MAX_SPEED.times(percentageBumpSpeed),
                                                null),

                                generalRobotCommands.driveToWithAngle(
                                                returnFromBump,
                                                kTightTolerance,
                                                bumpHeading),

                                shoot(generalRobotCommands, SHOOT_DELAY),

                                generalRobotCommands.getConveyorSubsystem().idle()
                                                .alongWith(generalRobotCommands.getFeederSubsytem().idle()),

                                // cycle 2

                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                : SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                                .times(-1),
                                                null),

                                generalRobotCommands.driveToWayPoint(
                                                frontTrench,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                                                : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                                isLeft
                                                                ? (isBlue ? SwerveConstants.MAX_SPEED
                                                                                : SwerveConstants.MAX_SPEED.times(-1))
                                                                : (isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                                                                : SwerveConstants.MAX_SPEED)),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake2,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPointWithAngle(
                                                                                finishIntake2,
                                                                                4.5,
                                                                                intakeHeading,
                                                                                null,
                                                                                intakeVelocity)),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())));
        }

        public static Command sideAuton2Sweep(GeneralRobotCommands generalRobotCommands, boolean isLeft) {
                var alliance = DriverStation.getAlliance();
                boolean isBlue;
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                        isBlue = true;
                } else {
                        isBlue = false;
                }

                Translation2d backup = sideRelative(WaypointConstants.BLUE_1_BACKUP, isBlue, isLeft);
                Translation2d runup = sideRelative(WaypointConstants.BLUE_1_RUNUP, isBlue, isLeft);
                Translation2d frontTrench = sideRelative(WaypointConstants.BLUE_1_FRONT_TRENCH, isBlue, isLeft);
                Translation2d beginIntake = sideRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue, isLeft);
                Translation2d beginIntake2 = sideRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE_2, isBlue, isLeft);
                Translation2d finishIntake = sideRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue, isLeft);
                Translation2d finishIntake2 = sideRelative(WaypointConstants.BLUE_1_FINISH_INTAKE_2, isBlue, isLeft);
                Translation2d prepareBump = sideRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue, isLeft);
                Translation2d returnFromBump = sideRelative(WaypointConstants.BLUE_1_RETURN, isBlue, isLeft);
                Translation2d shootPose = sideRelative(WaypointConstants.BLUE_1_SHOOT, isBlue, isLeft);
                Translation2d prepareDepot = sideRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue,
                                isLeft);
                Translation2d depotIntake = sideRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue, isLeft);
                Translation2d finalPos = sideRelative(WaypointConstants.BLUE_1_FINAL, isBlue, isLeft);

                Rotation2d forwardHeading = isBlue ? kForward : kBackward;
                Rotation2d backwardHeading = isBlue ? kBackward : kForward;

                Rotation2d intakeHeading = isLeft
                                ? (isBlue ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90))
                                : (isBlue ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90));

                Rotation2d bumpHeading = isLeft
                                ? (isBlue ? kBumpHeadingBottomRight : kBumpHeadingTopLeft)
                                : (isBlue ? kBumpHeadingTopRight : kBumpHeadingBottomLeft);

                double percentageIntakeSpeed = 0.4;
                double percentageBumpSpeed = 0.5;

                LinearVelocity intakeVelocity = isLeft
                                ? (isBlue ? SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed).times(-1)
                                                : SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed))
                                : (isBlue ? SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed)
                                                : SwerveConstants.MAX_SPEED.times(percentageIntakeSpeed).times(-1));

                final double SHOOT_DELAY = 4.5;

                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        Translation2d start = sideRelative(WaypointConstants.BLUE_1_START, isBlue,
                                                        isLeft);

                                        generalRobotCommands.getSwerveSubsystem()
                                                        .resetOdometry(new Pose2d(start, bumpHeading));
                                }),

                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                : SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                                .times(-1),
                                                null),

                                // generalRobotCommands.driveToWayPoint(
                                // frontTrench,
                                // 1,
                                // isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                // : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                // isLeft
                                // ? (isBlue ? SwerveConstants.MAX_SPEED
                                // : SwerveConstants.MAX_SPEED.times(-1))
                                // : (isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                // : SwerveConstants.MAX_SPEED)),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPointWithAngle(
                                                                                finishIntake,
                                                                                4.5, // 4.5
                                                                                intakeHeading,
                                                                                null,
                                                                                intakeVelocity),
                                                                generalRobotCommands.driveToWithAngle(
                                                                                prepareBump,
                                                                                kTightTolerance,
                                                                                bumpHeading)),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())),

                                generalRobotCommands.driveToWayPoint(
                                                returnFromBump,
                                                0.5,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(percentageBumpSpeed).times(-1)
                                                                : SwerveConstants.MAX_SPEED.times(percentageBumpSpeed),
                                                null),

                                generalRobotCommands.driveToWithAngle(
                                                returnFromBump,
                                                kTightTolerance,
                                                bumpHeading),

                                shoot(generalRobotCommands, SHOOT_DELAY),

                                generalRobotCommands.getConveyorSubsystem().idle()
                                                .alongWith(generalRobotCommands.getFeederSubsytem().idle()),

                                // cycle 2

                                generalRobotCommands.driveToWayPoint(
                                                runup,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                : SwerveConstants.MAX_SPEED.times(percentageBumpSpeed)
                                                                                .times(-1),
                                                null),

                                generalRobotCommands.driveToWayPoint(
                                                frontTrench,
                                                1,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(0.6)
                                                                : SwerveConstants.MAX_SPEED.times(0.6).times(-1),
                                                isLeft
                                                                ? (isBlue ? SwerveConstants.MAX_SPEED
                                                                                : SwerveConstants.MAX_SPEED.times(-1))
                                                                : (isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                                                                : SwerveConstants.MAX_SPEED)),

                                Commands.deadline(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                beginIntake2,
                                                                                kLooseTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPointWithAngle(
                                                                                finishIntake2,
                                                                                4.5, // 4.5
                                                                                intakeHeading,
                                                                                null,
                                                                                intakeVelocity),
                                                                generalRobotCommands.driveToWithAngle(
                                                                                prepareBump,
                                                                                kMedTolerance,
                                                                                bumpHeading)),
                                                Commands.waitSeconds(0.1)
                                                                .andThen(generalRobotCommands.intakeCommand())),
                                generalRobotCommands.driveToWayPoint(
                                                returnFromBump,
                                                0.5,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(percentageBumpSpeed).times(-1)
                                                                : SwerveConstants.MAX_SPEED.times(percentageBumpSpeed),
                                                null),

                                generalRobotCommands.driveToWithAngle(
                                                returnFromBump,
                                                kTightTolerance,
                                                bumpHeading),

                                shoot(generalRobotCommands, SHOOT_DELAY));
        }

        public static Translation2d sideRelative(Translation2d pose, boolean isBlue, boolean isLeft) {
                // first apply alliance flip
                double x = pose.getX();
                double y = pose.getY();

                if (!isBlue) {
                        x = LandMarks.kFieldLength.in(Meter) - x;
                        y = LandMarks.kFieldWidth.in(Meter) - y;
                }

                // then apply side mirror (only for right side)
                if (!isLeft) {
                        y = LandMarks.kFieldWidth.in(Meter) - y;
                }

                return new Translation2d(x, y);
        }

        public static Translation2d mirrorSide(Translation2d point) {
                return new Translation2d(
                                point.getX(),
                                LandMarks.kFieldWidth.magnitude() - point.getY());
        }

        private static Command shoot(GeneralRobotCommands generalRobotCommands, double seconds) {
                return new ParallelDeadlineGroup(
                                Commands.waitSeconds(seconds),
                                Commands.waitSeconds(0.5)
                                                .andThen(generalRobotCommands.feedCommand()),
                                generalRobotCommands.prepareShooter());
        }

        public static Command rightShiftToMiddle(GeneralRobotCommands generalRobotCommands) {
                var alliance = DriverStation.getAlliance();
                boolean isBlue;
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                        isBlue = true;
                } else {
                        isBlue = false;
                }
                Translation2d backup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_BACKUP, isBlue));
                Translation2d runup = mirrorSide(allianceRelative(WaypointConstants.BLUE_1_RUNUP, isBlue));
                Translation2d beginIntake = allianceRelative(WaypointConstants.BLUE_1_BEGIN_INTAKE, isBlue);
                Translation2d finishIntake = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE, isBlue);
                Translation2d finishIntake2 = allianceRelative(WaypointConstants.BLUE_1_FINISH_INTAKE2, isBlue);
                Translation2d prepareBump = allianceRelative(WaypointConstants.BLUE_1_PREPARE_BUMP, isBlue);
                Translation2d returnFromBump = allianceRelative(WaypointConstants.BLUE_1_RETURN, isBlue);
                Translation2d shootPose = allianceRelative(WaypointConstants.BLUE_1_SHOOT, isBlue);
                Translation2d prepareDepot = allianceRelative(WaypointConstants.BLUE_1_PREPARE_DEPOT_INTAKE, isBlue);
                Translation2d depotIntake = allianceRelative(WaypointConstants.BLUE_1_DEPOT_INTAKE, isBlue);
                Translation2d temp = mirrorSide(allianceRelative(WaypointConstants.TEMP, isBlue));
                Translation2d shootNoIntake = mirrorSide(allianceRelative(WaypointConstants.SHOOT_NO_INTAKE, isBlue));

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
                                        Translation2d start = mirrorSide(
                                                        allianceRelative(WaypointConstants.BLUE_1_START, isBlue));
                                        Rotation2d startHeading = isBlue ? kForward : kBackward;

                                        generalRobotCommands.getSwerveSubsystem().getSwerveDrive()
                                                        .resetOdometry(new Pose2d(start, startHeading));

                                }),

                                new WaitCommand(6),

                                generalRobotCommands.driveToWithAngle(shootNoIntake,
                                                kTightTolerance,
                                                forwardHeading),

                                new ParallelCommandGroup(generalRobotCommands.spinUpShooterCommand(),
                                                Commands.waitSeconds(3)
                                                                .andThen(generalRobotCommands.getConveyorSubsystem()
                                                                                .runCommand()
                                                                                .alongWith(generalRobotCommands
                                                                                                .getFeederSubsytem()
                                                                                                .runCommand()))));
        }

        public static Command testbackwardsAuton(GeneralRobotCommands generalRobotCommands) {
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
                                generalRobotCommands.getShooterSubsystem().runCommand(RPM.of(5300)),
                                generalRobotCommands.driveToWayPoint(
                                                forwardPoint,
                                                kTightTolerance,
                                                isBlue ? SwerveConstants.MAX_SPEED.times(-1)
                                                                : SwerveConstants.MAX_SPEED,
                                                null));
        }

        public static Command testRotateAuton(GeneralRobotCommands generalRobotCommands) {
                boolean isBlue = DriverStation.getAlliance()
                                .map(alliance -> alliance == DriverStation.Alliance.Blue)
                                .orElse(true);

                Translation2d start = allianceRelative(WaypointConstants.BLUE_1_START, isBlue);
                Rotation2d startHeading = isBlue ? kForward : kBackward;
                Rotation2d rotatedHeading = startHeading.plus(Rotation2d.fromDegrees(90));

                return new SequentialCommandGroup(
                                new InstantCommand(() -> generalRobotCommands.getSwerveSubsystem()
                                                .resetOdometry(new Pose2d(start, startHeading))),

                                generalRobotCommands.driveToWithAngle(
                                                start,
                                                kTightTolerance,
                                                rotatedHeading));
        }

        public static Command testMiddleAuton(GeneralRobotCommands generalRobotCommands) {
                var alliance = DriverStation.getAlliance();
                boolean isBlue;
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                        isBlue = true;
                } else {
                        isBlue = false;
                }

                Translation2d towerPose = allianceRelative(WaypointConstants.BLUE_2_TOWER, isBlue);
                Translation2d shootPose = allianceRelative(WaypointConstants.BLUE_2_SHOOT, isBlue);
                Translation2d prepareDepot = allianceRelative(WaypointConstants.BLUE_2_PREPARE_DEPOT, isBlue);
                Translation2d prepareDepot2 = allianceRelative(WaypointConstants.BLUE_2_PREPARE_DEPOT_2, isBlue);
                Translation2d depotIntake = allianceRelative(WaypointConstants.BLUE_2_DEPOT_INTAKE, isBlue);
                Translation2d finishDepot = allianceRelative(WaypointConstants.BLUE_2_FINISH_DEPOT, isBlue);

                Rotation2d forwardHeading = isBlue ? kForward : kBackward;
                Rotation2d backwardHeading = isBlue ? kBackward : kForward;
                Rotation2d intakeHeading = isBlue ? kIntakeHeading : Rotation2d.fromDegrees(90);

                return new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                        Translation2d start = mirrorSide(
                                                        allianceRelative(WaypointConstants.BLUE_2_START, isBlue));
                                        Rotation2d startHeading = isBlue ? kBackward : kForward;

                                        generalRobotCommands.getSwerveSubsystem().getSwerveDrive()
                                                        .resetOdometry(new Pose2d(start, startHeading));

                                }),

                                generalRobotCommands.driveTo(
                                                towerPose,
                                                kMedTolerance),

                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                generalRobotCommands.driveToWithAngle(
                                                                                prepareDepot,
                                                                                0.5,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWithAngle(
                                                                                prepareDepot2,
                                                                                kTightTolerance,
                                                                                intakeHeading),
                                                                generalRobotCommands.driveToWayPoint(
                                                                                depotIntake,
                                                                                kMedTolerance,
                                                                                null,
                                                                                isBlue ? SwerveConstants.MAX_SPEED
                                                                                                .times(0.3).times(-1)
                                                                                                : SwerveConstants.MAX_SPEED
                                                                                                                .times(0.3))),
                                                Commands.waitSeconds(1).andThen(generalRobotCommands.intakeCommand())),

                                generalRobotCommands.driveTo(
                                                finishDepot,
                                                0.5),

                                generalRobotCommands.driveToWithAngle(
                                                shootPose,
                                                kMedTolerance,
                                                forwardHeading),
                                generalRobotCommands.aimSwerveToHubCommand(),
                                shoot(generalRobotCommands, 10));
                // shootWhenReady(generalRobotCommands, 1000));
        }

}
