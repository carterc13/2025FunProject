// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {

  public static Distance applyX(Distance x) {
    return shouldFlip() ? FieldConstants.fieldLength.minus(x) : x;
  }

  public static Distance applyY(Distance y) {
    return shouldFlip() ? FieldConstants.fieldWidth.minus(y) : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getMeasureX()), applyY(translation.getMeasureY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getMeasureX()),
        applyY(translation.getMeasureY()),
        translation.getMeasureZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip()
        ? rotation.rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(180)))
        : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Pose3d apply(Pose3d pose) {
    return shouldFlip()
        ? new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
