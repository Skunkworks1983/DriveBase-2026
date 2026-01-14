// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

  private final CollectorIO collectorIO;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

  public Collector(CollectorIO collectorIO) {
    this.collectorIO = collectorIO;
  }

  // motor ID 11

  @Override
  public void periodic() {
    collectorIO.updateInputs(inputs);
  }

  public void setCollectorVelocity(double velocity) {
    collectorIO.setCollectorVelocity(velocity);
  }
}
