package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashSet;
import java.util.LinkedHashSet;

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

  public record ScoringPose(ReefFace face, BranchSide side) {
    @Override
    public String toString() {
      return face.name() + " " + side.name();
    }
  }

  public static HashSet<ScoringPose> getAllScoringPoses() {
    HashSet<ScoringPose> scoringPoses =
        new LinkedHashSet<
            ScoringPose>(); // Using a linked set to keep items ordered for auto select display
    for (ReefFace face : ReefFace.values()) {
      scoringPoses.add(new ScoringPose(face, BranchSide.LEFT));
      scoringPoses.add(new ScoringPose(face, BranchSide.RIGHT));
    }
    return scoringPoses;
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

  public static Pose2d getReefAprilTagPose(ReefFace face) {
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
    return tagPose.toPose2d();
  }

  public static Pose2d getReefAprilTagPose(ScoringPose scoringPose) {
    return getReefAprilTagPose(scoringPose.face);
  }

  public static Pose2d getReefScorePose(ScoringPose pose) {
    if (pose.side == BranchSide.LEFT) {
      return getReefAprilTagPose(pose).transformBy(leftReefScoringTransform);
    } else {
      return getReefAprilTagPose(pose).transformBy(rightReefScoringTransform);
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

  // REMENANTS OF AN ATTEMPT OF SWITCHING TO MANUAL PATHS AT END
  // This is the location pathfinding will go to before switching to a manual path
  // public static Transform2d reefPathfindingOfset = new Transform2d(1, 0, new Rotation2d());
  // public static HashMap<ScoringPose, PathPlannerPath> coralFinalAlignmentPaths = new HashMap<>();

  // public static void initCoralFinalAlignmentPaths() {
  //   for (ReefFace loc : ReefFace.values()) {
  //     for (BranchSide side : BranchSide.values()) {
  //       ScoringPose scoringId = new ScoringPose(loc, side);
  //       Pose2d startPose = getReefAprilTagPose(scoringId).transformBy(reefPathfindingOfset);
  //       Pose2d endPose = getReefScorePose(scoringId);

  //       // TODO: Replace with actual constraints
  //       PathConstraints constraints = PathConstraints.unlimitedConstraints(12);

  //       PathPlannerPath path =
  //           new PathPlannerPath(
  //               PathPlannerPath.waypointsFromPoses(startPose, endPose),
  //               constraints,
  //               new IdealStartingState(3.3, endPose.getRotation()),
  //               new GoalEndState(0, endPose.getRotation()));

  //       coralFinalAlignmentPaths.put(scoringId, path);
  //     }
  //   }
  // }

  // public static Pose2d getReefPathfindingPose(ScoringPose pose) {
  //   return getReefAprilTagPose(pose).transformBy(reefPathfindingOfset);
  // }
}
