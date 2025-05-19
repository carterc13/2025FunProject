// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Add your docs here. */
public class PoseComputer {
  private static boolean redalliance = false;

  public static void setRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      redalliance = alliance.get() == DriverStation.Alliance.Red;
    } else {
      DataLogManager.log("ERROR: Alliance not found. Defaulting to Blue");
      redalliance = false;
    }
  }

  public static boolean isRightSource(double y) {
    return redalliance
        ? y > FieldConstants.fieldWidth.in(Meters) / 2
        : y < FieldConstants.fieldWidth.in(Meters) / 2;
  }
}
