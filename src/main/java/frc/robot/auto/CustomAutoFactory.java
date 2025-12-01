package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants.AutoStartPoses;
import frc.robot.util.FieldConstants.CoralScoreLocation;
import frc.robot.util.FieldConstants.CoralStation;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CustomAutoFactory {
  private static LoggedDashboardChooser<AutoStartPoses> startPoseChooser;
  private static LoggedDashboardChooser<CoralStation> pickup1Chooser;
  private static LoggedDashboardChooser<CoralStation> pickup2Chooser;
  private static LoggedDashboardChooser<CoralStation> pickup3Chooser;
  private static LoggedDashboardChooser<CoralScoreLocation> score1Chooser;
  private static LoggedDashboardChooser<CoralScoreLocation> score2Chooser;
  private static LoggedDashboardChooser<CoralScoreLocation> score3Chooser;
  private static LoggedDashboardChooser<CoralScoreLocation> score4Chooser;

  public static void registerCustomAutoChoices() {
    startPoseChooser = new LoggedDashboardChooser<>("Custom Auto/Start Pose");
    startPoseChooser.addDefaultOption("Left", AutoStartPoses.LEFT);
    startPoseChooser.addOption("Middle", AutoStartPoses.MIDDLE);
    startPoseChooser.addOption("Right", AutoStartPoses.RIGHT);

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

  private static void registerScoreLocations(
      LoggedDashboardChooser<CoralScoreLocation> scoreChoser) {
    scoreChoser.addDefaultOption("AB LEFT", CoralScoreLocation.AB_LEFT);
    scoreChoser.addOption("AB RIGHT", CoralScoreLocation.AB_RIGHT);
    scoreChoser.addOption("CD LEFT", CoralScoreLocation.CD_LEFT);
    scoreChoser.addOption("CD RIGHT", CoralScoreLocation.CD_RIGHT);
    scoreChoser.addOption("EF LEFT", CoralScoreLocation.EF_LEFT);
    scoreChoser.addOption("EF RIGHT", CoralScoreLocation.EF_RIGHT);
    scoreChoser.addOption("GH LEFT", CoralScoreLocation.GH_LEFT);
    scoreChoser.addOption("GH RIGHT", CoralScoreLocation.GH_RIGHT);
    scoreChoser.addOption("IJ LEFT", CoralScoreLocation.IJ_LEFT);
    scoreChoser.addOption("IJ RIGHT", CoralScoreLocation.IJ_RIGHT);
    scoreChoser.addOption("KL LEFT", CoralScoreLocation.KL_LEFT);
    scoreChoser.addOption("KL RIGHT", CoralScoreLocation.KL_RIGHT);
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
