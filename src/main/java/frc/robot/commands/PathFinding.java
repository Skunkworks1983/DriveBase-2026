package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.BranchSide;
import frc.robot.util.FieldConstants.ReefFace;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class PathFinding {
    public static Command pathfindToReefScorePose(ReefFace face, BranchSide side) {
        Pose2d targetPose = FieldConstants.getScoringLocBlue(face, side);
        Logger.recordOutput("Pathfinding/Target Pose", targetPose);

        // Path find flipped properly mirrors field for alliances
        return AutoBuilder.pathfindToPoseFlipped(
                targetPose, PathFindingConstants.buildPathConstraints(), 0.0 // Goal end velocity in meters/sec
        );
    }

    // This is not a final implementation, just for testing (generating this command
    // just in time is
    // expensive, but quick way to do elastic isLeft)
    public static Command pathfindToReefScorePose(
            ReefFace face, BooleanSupplier isLeft, Drive drive) {
        return Commands.defer(
                () -> pathfindToReefScorePose(
                        face, isLeft.getAsBoolean() ? BranchSide.LEFT : BranchSide.RIGHT),
                Set.of(drive));
    }

    /**
     * Use PathPlanner pathfinding to generate a path to the coral station
     * 
     * Command creation is deferred so that current pose is fetched just in time
     * 
     * To ensure approach with proper rotation, a path is found to a point offset
     * from the coral station, then a path is followed in
     * 
     * @return
     */
    public static Command pathfindToNearestCoralStation(Drive drive) {
        return Commands.defer(() -> {
            Pose2d targetPose = FieldConstants.getNearestCoralStationBlue(drive.getPose());
            Logger.recordOutput("Pathfinding/Target Pose", targetPose);
            List<Waypoint> pathPoints = PathPlannerPath.waypointsFromPoses(
                    targetPose.transformBy(FieldConstants.coralStationCollectPathfindTransform),
                    targetPose.transformBy(FieldConstants.coralStationCollectTransform));

            PathPlannerPath finalApproach = new PathPlannerPath(pathPoints, PathFindingConstants.buildPathConstraints(),
                    new IdealStartingState(3.3, targetPose.getRotation()), new GoalEndState(0, targetPose.getRotation()));

            // Path find flipped properly mirrors field for alliances
            return AutoBuilder.pathfindThenFollowPath(
                    finalApproach, PathFindingConstants.buildPathConstraints());
        }, Set.of(drive));

    }
}
