package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.BranchSide;
import frc.robot.util.FieldConstants.CoralStation;
import frc.robot.util.FieldConstants.ReefFace;
import frc.robot.util.FieldConstants.ScoringPose;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CustomAutoFactory {
  private static LoggedDashboardChooser<CoralStation> pickup1Chooser;
  private static LoggedDashboardChooser<CoralStation> pickup2Chooser;
  private static LoggedDashboardChooser<CoralStation> pickup3Chooser;
  private static LoggedDashboardChooser<ScoringPose> score1Chooser;
  private static LoggedDashboardChooser<ScoringPose> score2Chooser;
  private static LoggedDashboardChooser<ScoringPose> score3Chooser;
  private static LoggedDashboardChooser<ScoringPose> score4Chooser;

  public static void registerCustomAutoChoices() {
    pickup1Chooser = new LoggedDashboardChooser<>("Custom Auto/Pickup 1 Location");
    registerPickupLocations(pickup1Chooser);
    pickup2Chooser = new LoggedDashboardChooser<>("Custom Auto/Pickup 2 Location");
    registerPickupLocations(pickup2Chooser);
    pickup3Chooser = new LoggedDashboardChooser<>("Custom Auto/Pickup 3 Location");
    registerPickupLocations(pickup3Chooser);

    score1Chooser = new LoggedDashboardChooser<>("Custom Auto/Score 1 Location");
    registerScoreLocations(score1Chooser);
    score2Chooser = new LoggedDashboardChooser<>("Custom Auto/Score 2 Location");
    registerScoreLocations(score2Chooser);
    score3Chooser = new LoggedDashboardChooser<>("Custom Auto/Score 3 Location");
    registerScoreLocations(score3Chooser);
    score4Chooser = new LoggedDashboardChooser<>("Custom Auto/Score 4 Location");
    registerScoreLocations(score4Chooser);
  }

  private static void registerPickupLocations(LoggedDashboardChooser<CoralStation> pickupChooser) {
    pickupChooser.addDefaultOption("Coral Station Left", CoralStation.CORAL_STATION_LEFT);
    pickupChooser.addDefaultOption("Coral Station Right", CoralStation.CORAL_STATION_RIGHT);
  }

  private static void registerScoreLocations(LoggedDashboardChooser<ScoringPose> scoreChoser) {
    FieldConstants.getAllScoringPoses()
        .forEach((pose) -> scoreChoser.addOption(pose.toString(), pose));
    ScoringPose defaultPose = new ScoringPose(ReefFace.AB, BranchSide.LEFT);
    scoreChoser.addDefaultOption(defaultPose.toString(), defaultPose);
  }

  public static Command getCustomAuto(Drive drive) {
    return Commands.defer(
        () ->
            new SequentialCommandGroup(
                PathFinding.pathfindToReefScorePose(score1Chooser.get()),
                PathFinding.pathfindToCoralStation(pickup1Chooser.get()),
                PathFinding.pathfindToReefScorePose(score2Chooser.get()),
                PathFinding.pathfindToCoralStation(pickup2Chooser.get()),
                PathFinding.pathfindToReefScorePose(score3Chooser.get()),
                PathFinding.pathfindToCoralStation(pickup3Chooser.get()),
                PathFinding.pathfindToReefScorePose(score4Chooser.get())),
        Set.of(drive));
  }

  private static void VisualizeCustomAuto() {}
}
