package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.LoggedTunableNumber;

public class PathFindingConstants {
    public static LoggedTunableNumber maxVelocityMPS = new LoggedTunableNumber("PathFinding/Max Velocity m/s", 3.0);
    public static LoggedTunableNumber maxAccelerationMPSPS = new LoggedTunableNumber(
            "PathFinding/Max Acceleration m/s^2", 4.0);
    public static LoggedTunableNumber maxAngularVelocityDPS = new LoggedTunableNumber(
            "PathFinding/Max Angular Velocity deg/s", 540);
    public static LoggedTunableNumber maxAngularAccelerationDPSPS = new LoggedTunableNumber(
            "PathFinding/Max Angular Acceleration deg/s^2", 720);

    public static PathConstraints buildPathConstraints() {
        // Get constraints just in time to allow dynamic tuning
        LinearVelocity maxVelocity = LinearVelocity.ofBaseUnits(PathFindingConstants.maxVelocityMPS.get(),
                MetersPerSecond);
        LinearAcceleration maxAcceleration = LinearAcceleration.ofBaseUnits(
                PathFindingConstants.maxAccelerationMPSPS.get(), MetersPerSecondPerSecond);
        AngularVelocity maxAngularVelocity = AngularVelocity.ofBaseUnits(
                PathFindingConstants.maxAngularVelocityDPS.get(), DegreesPerSecond);
        AngularAcceleration maxAngularAcceleration = AngularAcceleration.ofBaseUnits(
                PathFindingConstants.maxAngularAccelerationDPSPS.get(), DegreesPerSecondPerSecond);

        // Create the constraints to use while pathfinding
        return new PathConstraints(
                maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);
    }
}
