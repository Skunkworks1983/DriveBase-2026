package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.BranchSide;
import frc.robot.util.FieldConstants.CoralScoreLocation;
import frc.robot.util.FieldConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefFace;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Container for path finding commands
 *
 * @author Eddy W
 */
public class PathFinding {
  public static Command pathfindToPose2d(Pose2d targetPose) {
    Logger.recordOutput("Pathfinding/Target Pose", targetPose);

    // Path find flipped properly mirrors field for alliances
    return AutoBuilder.pathfindToPoseFlipped(
        targetPose,
        PathFindingConstants.buildPathConstraints(),
        0.0 // Goal end velocity in meters/sec
        );
  }

  /**
   * @param face The face of the reef to score on
   * @param branch The reef branch to score on
   * @return A command that pathfinds to the specified reef scoring pose
   */
  public static Command pathfindToReefScorePose(ReefFace face, BranchSide branch) {
    Pose2d targetPose = FieldConstants.getScoringLocBlue(face, branch);
    return pathfindToPose2d(targetPose);
  }

  public static Command pathfindToReefScorePose(CoralScoreLocation scoreLoc) {
    Pose2d targetPose = FieldConstants.getScoringLocBlue(scoreLoc);

    return pathfindToPose2d(targetPose);
  }

  /**
   * @param face The face of the reef to score on
   * @param isLeft A supplier that is gotten just in time of the branch side to score on (allows for
   *     a toggle)
   * @return A command that pathfinds to the specified reef scoring pose
   */
  public static Command pathfindToReefScorePose(
      ReefFace face, BooleanSupplier isLeft, Drive drive) {
    return Commands.defer(
        () ->
            pathfindToReefScorePose(
                face, isLeft.getAsBoolean() ? BranchSide.LEFT : BranchSide.RIGHT),
        Set.of(drive));
  }

  /**
   * @return A command that pathfinds to a small distance from the nearest coral station, then
   *     follows a specific final alignment path to ensure ending with correct orientation
   */
  public static Command pathfindToNearestCoralStation(Drive drive) {
    return Commands.defer(
        () -> {
          CoralStation targetStation = FieldConstants.getNearestCoralStationBlue(drive.getPose());
          Logger.recordOutput("Pathfinding/Target Coral Station", targetStation);

          return pathfindToCoralStation(targetStation);
        },
        Set.of(drive));
  }

  public static Command pathfindToCoralStation(CoralStation coralStation) {
    Pose2d targetPose = FieldConstants.getCoralStationAprilTagLocBlue(coralStation);
    Logger.recordOutput(
        "Pathfinding/Target Pose",
        targetPose.transformBy(FieldConstants.coralStationCollectPathfindTransform));
    List<Waypoint> pathPoints =
        PathPlannerPath.waypointsFromPoses(
            targetPose.transformBy(FieldConstants.coralStationCollectPathfindTransform),
            targetPose.transformBy(FieldConstants.coralStationCollectTransform));

    PathPlannerPath finalApproach =
        new PathPlannerPath(
            pathPoints,
            PathFindingConstants.buildPathConstraints(),
            new IdealStartingState(3.3, targetPose.getRotation()),
            new GoalEndState(0, targetPose.getRotation()));

    // Path find flipped properly mirrors field for alliances
    return AutoBuilder.pathfindThenFollowPath(
        finalApproach, PathFindingConstants.buildPathConstraints());
  }
}
