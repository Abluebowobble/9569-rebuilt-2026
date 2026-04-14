
package frc.robot;

import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LandMarks {
    public static final AprilTagFieldLayout K_APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance kFieldLength = Meters.of(K_APRIL_TAG_FIELD_LAYOUT.getFieldLength());
    public static final Distance kFieldWidth = Meters.of(K_APRIL_TAG_FIELD_LAYOUT.getFieldWidth());

    public static final Distance kAllianceFieldLength = Inches.of(182.105);
    public static final Optional<Alliance> kAlliance = DriverStation.getAlliance();

    public static final Distance kTowerX = Inches.of(20.223);

    // 11.34
    public static final Distance kTowerOffset = Inches.of(11.34);

    public static Translation2d hubPosition() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return new Translation2d(kAllianceFieldLength, Inches.of(158.845));
        } else {
            return new Translation2d(kFieldLength.minus(kAllianceFieldLength), Inches.of(158.845));
        }
    }

    public static Distance allianceHubCentreX() {
        if (kAlliance.isPresent() && kAlliance.get() == Alliance.Blue) {
            return kAllianceFieldLength.div(2);
        }

        return kFieldLength.minus(kAllianceFieldLength).div(2);
    }

    public static Translation2d blueTowerPosition() {
        return new Translation2d(Inches.of(20.223), Inches.of(158.845).minus(kTowerOffset));
    }

    public static Translation2d redTowerPosition() {
        return new Translation2d(kFieldLength.minus(Inches.of(20.223)), Inches.of(158.845).plus(kTowerOffset));
    }
}
