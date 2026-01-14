// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;

public class CollectorCommand extends Command {

  double velocity;
  Collector collector;

  public CollectorCommand(Collector collector, double velocity) {

    this.velocity = velocity;
    this.collector = collector;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    collector.setCollectorVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    collector.setCollectorVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
