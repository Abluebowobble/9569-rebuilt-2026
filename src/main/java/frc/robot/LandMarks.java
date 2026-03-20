package frc.robot;

import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LandMarks {
    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = layout.getFieldLength();
    public static final double fieldWidth = layout.getFieldWidth();

    public static final double allianceFieldLength = 182.105;
    public static final Optional<Alliance> alliance = DriverStation.getAlliance();

    public static Translation2d hubPosition() {
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }

        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }

    public static double allianceSectionLength() {
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return allianceFieldLength;
        }

        return fieldLength - allianceFieldLength;
    }
}
