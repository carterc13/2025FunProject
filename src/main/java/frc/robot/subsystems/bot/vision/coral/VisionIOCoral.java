// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.bot.vision.coral;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.simulation.VisionTargetSim;

public interface VisionIOCoral {
  @AutoLog
  public static class VisionIOCoralInputs {
    boolean connected = false;
    Pose3d[] coralPoseEstimates = new Pose3d[] {};
    Pose3d[] acceptedCoralPoseEstimates = new Pose3d[] {};
    Pose3d[] rejectedCoralPoseEstimates = new Pose3d[] {};
  }

  default void updateInputs(VisionIOCoralInputs inputs) {}

  default void updateModels(VisionTargetSim[] models) {}
}
