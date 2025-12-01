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
  public static enum ReefFace {
    AB,
    CD,
    EF,
    GH,
    IJ,
    KL
  }

  public static enum BranchSide {
    LEFT,
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

  public static Pose2d getNearestCoralStationBlue(Pose2d currPose) {
    // Left and right from driver station perspective
    Pose2d leftCoralStation = aprilTagLayout.getTagPose(13).get().toPose2d();
    Pose2d rightCoralStation = aprilTagLayout.getTagPose(12).get().toPose2d();

    // Get robots distance to each tag
    double distToLeft = currPose.getTranslation().getDistance(leftCoralStation.getTranslation());
    double distToRight = currPose.getTranslation().getDistance(rightCoralStation.getTranslation());

    // Return nearest coral station
    return (distToLeft > distToRight)
        ? (DriverStation.getAlliance().get() == Alliance.Blue
            ? rightCoralStation
            : leftCoralStation)
        : (DriverStation.getAlliance().get() == Alliance.Blue
            ? leftCoralStation
            : rightCoralStation);
  }
}
