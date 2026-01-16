// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;

public class CollectorCommand extends Command {

  double velocityOne;
  double velocityTwo;
  Collector collector;

  public CollectorCommand(Collector collector, double velocityOne, double velocityTwo) {

    this.velocityOne = velocityOne;
    this.velocityTwo = velocityTwo;
    this.collector = collector;

    addRequirements(collector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    collector.setCollectorVelocity(
      SmartDashboard.getNumber("/SmartDashboard/collector/motor1", 0),
      SmartDashboard.getNumber("/SmartDashboard/collector/motor2", 0));

      System.out.println(SmartDashboard.getNumber("/SmartDashboard/collector/motor1", 0));
      
  }

  @Override
  public void end(boolean interrupted) {
    collector.setCollectorVelocity(0,0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
