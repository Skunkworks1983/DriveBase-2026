package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
  /** Reef face following Game Manual labeling */
  public static enum ReefFace {
    AB,
    CD,
    EF,
    GH,
    IJ,
    KL
  }

  /**
   * Coral branch side
   *
   * <p>Left and right with respect to reef face
   */
  public static enum BranchSide {
    LEFT,
    RIGHT
  }

  /**
   * Autonomous pickup locations
   *
   * <p>Left and Right are from the driver station perspective
   */
  public static enum CoralStation {
    CORAL_STATION_LEFT,
    CORAL_STATION_RIGHT,
  }

  /**
   * Coral scoring locations
   *
   * <p>Reef faces follow game manual naming, left and right with respect to reef face
   */
  public static enum CoralScoreLocation {
    AB_LEFT,
    CD_LEFT,
    EF_LEFT,
    GH_LEFT,
    IJ_LEFT,
    KL_LEFT,
    AB_RIGHT,
    CD_RIGHT,
    EF_RIGHT,
    GH_RIGHT,
    IJ_RIGHT,
    KL_RIGHT
  }

  /**
   * This would not be done like this on an actual bot, but it works for a proof of concept
   *
   * <p>Left and right with respect to driver station
   */
  public static enum AutoStartPoses {
    LEFT,
    MIDDLE,
    RIGHT
  }

  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // All transforms are from the april tag at the given location
  public static Transform2d leftReefScoringTransform =
      new Transform2d(0.55, -0.165, new Rotation2d(Math.PI));
  public static Transform2d rightReefScoringTransform =
      new Transform2d(0.55, 0.165, new Rotation2d(Math.PI));
  public static Transform2d coralStationCollectPathfindTransform =
      new Transform2d(1, 0, new Rotation2d());
  public static Transform2d coralStationCollectTransform =
      new Transform2d(0.55, 0, new Rotation2d());

  public static Pose2d getScoringLocBlue(CoralScoreLocation scoreLoc) {
    switch (scoreLoc) {
      case AB_LEFT:
        return getScoringLocBlue(ReefFace.AB, BranchSide.LEFT);
      case CD_LEFT:
        return getScoringLocBlue(ReefFace.CD, BranchSide.LEFT);
      case EF_LEFT:
        return getScoringLocBlue(ReefFace.EF, BranchSide.LEFT);
      case GH_LEFT:
        return getScoringLocBlue(ReefFace.GH, BranchSide.LEFT);
      case IJ_LEFT:
        return getScoringLocBlue(ReefFace.IJ, BranchSide.LEFT);
      case KL_LEFT:
        return getScoringLocBlue(ReefFace.KL, BranchSide.LEFT);
      case AB_RIGHT:
        return getScoringLocBlue(ReefFace.AB, BranchSide.RIGHT);
      case CD_RIGHT:
        return getScoringLocBlue(ReefFace.CD, BranchSide.RIGHT);
      case EF_RIGHT:
        return getScoringLocBlue(ReefFace.EF, BranchSide.RIGHT);
      case GH_RIGHT:
        return getScoringLocBlue(ReefFace.GH, BranchSide.RIGHT);
      case IJ_RIGHT:
        return getScoringLocBlue(ReefFace.IJ, BranchSide.RIGHT);
      case KL_RIGHT:
        return getScoringLocBlue(ReefFace.KL, BranchSide.RIGHT);
      default:
        return null;
    }
  }

  public static Pose2d getScoringLocBlue(ReefFace face, BranchSide side) {
    Pose3d tagPose;
    switch (face) {
      case AB:
        tagPose = aprilTagLayout.getTagPose(18).get();
        break;
      case CD:
        tagPose = aprilTagLayout.getTagPose(17).get();
        break;
      case EF:
        tagPose = aprilTagLayout.getTagPose(22).get();
        break;
      case GH:
        tagPose = aprilTagLayout.getTagPose(21).get();
        break;
      case IJ:
        tagPose = aprilTagLayout.getTagPose(20).get();
        break;
      case KL:
        tagPose = aprilTagLayout.getTagPose(19).get();
        break;
      default:
        return null;
    }
    return getReefScoringPoseWithOffset(tagPose, side);
  }

  private static Pose2d getReefScoringPoseWithOffset(Pose3d tagPose, BranchSide side) {
    if (side == BranchSide.LEFT) {
      return tagPose.toPose2d().transformBy(leftReefScoringTransform);
    } else {
      return tagPose.toPose2d().transformBy(rightReefScoringTransform);
    }
  }

  public static CoralStation getNearestCoralStationBlue(Pose2d currPose) {
    // Left and right from driver station perspective
    Pose2d leftCoralStation = aprilTagLayout.getTagPose(13).get().toPose2d();
    Pose2d rightCoralStation = aprilTagLayout.getTagPose(12).get().toPose2d();

    // Get robots distance to each tag
    double distToLeft = currPose.getTranslation().getDistance(leftCoralStation.getTranslation());
    double distToRight = currPose.getTranslation().getDistance(rightCoralStation.getTranslation());

    // Return nearest coral station
    return (distToLeft < distToRight)
        ? (DriverStation.getAlliance().get() == Alliance.Blue
            ? CoralStation.CORAL_STATION_LEFT
            : CoralStation.CORAL_STATION_RIGHT)
        : (DriverStation.getAlliance().get() == Alliance.Blue
            ? CoralStation.CORAL_STATION_RIGHT
            : CoralStation.CORAL_STATION_LEFT);
  }

  public static Pose2d getCoralStationAprilTagLocBlue(CoralStation station) {
    switch (station) {
      case CORAL_STATION_LEFT:
        return aprilTagLayout.getTagPose(13).get().toPose2d();
      case CORAL_STATION_RIGHT:
        return aprilTagLayout.getTagPose(12).get().toPose2d();
      default:
        return null;
    }
  }
}
