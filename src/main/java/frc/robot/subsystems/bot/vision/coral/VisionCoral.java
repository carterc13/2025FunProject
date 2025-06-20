// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.bot.vision.coral;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PoseComputer;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

/** Subsystem that handles vision processing from multiple cameras for Coral */
public class VisionCoral extends SubsystemBase {
  private static final String VISION_PATH = "VisionCoral/Camera";

  private final VisionIOCoral[] io;
  private final VisionIOCoralInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private ArrayList<Pose3d> coralPoses = new ArrayList<Pose3d>() {};
  private ArrayList<Pose3d> rejectedCoralPoses = new ArrayList<Pose3d>() {};
  private ArrayList<Pose3d> acceptedCoralPoses = new ArrayList<Pose3d>() {};
  private Pose3d leftCoralPose = new Pose3d();
  private Pose3d rightCoralPose = new Pose3d();
  TargetModel targetModel =
      new TargetModel(
          Units.inchesToMeters(11.875), Units.inchesToMeters(4.5), Units.inchesToMeters(4.5));

  /**
   * Creates a new VisionCoral subsystem.
   *
   * @param io Array of VisionIOCoral interfaces for each camera
   */
  public VisionCoral(VisionIOCoral... io) {
    System.out.println("[Init] Creating VisionCoral");
    this.io = io;

    inputs = new VisionIOCoralInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOCoralInputsAutoLogged();
    }

    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(String.format("VisionCoral camera %d is disconnected.", i), AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    coralPoses = new ArrayList<Pose3d>() {};
    rejectedCoralPoses = new ArrayList<Pose3d>() {};
    acceptedCoralPoses = new ArrayList<Pose3d>() {};
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
      for (int v = 0; v < inputs[i].acceptedCoralPoseEstimates.length; v++) {
        acceptedCoralPoses.add(inputs[i].acceptedCoralPoseEstimates[v]);
      }
      for (int v = 0; v < inputs[i].rejectedCoralPoseEstimates.length; v++) {
        rejectedCoralPoses.add(inputs[i].rejectedCoralPoseEstimates[v]);
      }
      for (int v = 0; v < inputs[i].coralPoseEstimates.length; v++) {
        coralPoses.add(inputs[i].coralPoseEstimates[v]);
      }
      Logger.processInputs(VISION_PATH + i, inputs[i]);
    }
    ArrayList<Pose3d> temp = averageGroupedPoses(acceptedCoralPoses);
    Logger.recordOutput("VisionCoral/Coral Pose Estimates", arrayToOther(coralPoses));
    Logger.recordOutput("VisionCoral/Accepted Coral Poses Average", arrayToOther(temp));
    Logger.recordOutput("VisionCoral/Accepted Coral Poses", arrayToOther(acceptedCoralPoses));
    Logger.recordOutput("VisionCoral/Rejected Coral Poses", arrayToOther(rejectedCoralPoses));
    if (temp.size() == 1) {
      if (PoseComputer.isRightSource(() -> temp.get(0).getY())) {
        rightCoralPose = temp.get(0);
      } else {
        leftCoralPose = temp.get(0);
      }
    } else if (temp.size() == 2) {
      if (PoseComputer.isRightSource(() -> temp.get(0).getY())) {
        rightCoralPose = temp.get(0);
        leftCoralPose = temp.get(1);
      } else {
        rightCoralPose = temp.get(1);
        leftCoralPose = temp.get(0);
      }
    }
    Logger.recordOutput("VisionCoral/Accepted Right Coral Pose", rightCoralPose);
    Logger.recordOutput("VisionCoral/Accepted Left Coral Pose", leftCoralPose);
  }

  public Pose2d getRightCoralPose() {
    return new Pose2d(
        rightCoralPose.getTranslation().toTranslation2d(),
        rightCoralPose.getRotation().toRotation2d());
  }

  public Pose2d getLeftCoralPose() {
    return new Pose2d(
        leftCoralPose.getTranslation().toTranslation2d(),
        leftCoralPose.getRotation().toRotation2d());
  }

  private ArrayList<Pose3d> averageGroupedPoses(ArrayList<Pose3d> poses) {
    ArrayList<Pose3d> badPoses = new ArrayList<Pose3d>();
    ArrayList<Pose3d> goodPoses = new ArrayList<Pose3d>();
    if (!poses.isEmpty()) {
      goodPoses.add(poses.get(poses.size() - 1));
      Pose3d startPose = poses.get(poses.size() - 1);
      poses.remove(poses.size() - 1);
      for (int i = 0; i < poses.size(); i++) {
        if (startPose.getTranslation().getDistance(poses.get(i).getTranslation()) > 0.6) {
          badPoses.add(poses.get(i));
        } else {
          goodPoses.add(poses.get(i));
        }
      }
      ArrayList<Pose3d> newGoodPoses = new ArrayList<Pose3d>();
      newGoodPoses.add(averragePoses(goodPoses));
      if (badPoses.isEmpty() || badPoses.size() == 0) {
        return newGoodPoses;
      } else if (badPoses.size() == 1) {
        newGoodPoses.add(badPoses.get(0));
        return newGoodPoses;
      } else {
        newGoodPoses.addAll(averageGroupedPoses(badPoses));
        return newGoodPoses;
      }
    }
    return new ArrayList<Pose3d>();
  }

  private Pose3d averragePoses(ArrayList<Pose3d> poses) {
    Pose3d total = new Pose3d();
    for (int i = 0; i < poses.size(); i++) {
      total =
          new Pose3d(
              poses.get(i).getTranslation().plus(total.getTranslation()),
              total.getRotation().plus(poses.get(i).getRotation()));
    }
    return total.div(poses.size());
  }

  private Pose3d[] arrayToOther(ArrayList<Pose3d> poses) {
    Pose3d[] newPoses = new Pose3d[poses.size()];
    for (int i = 0; i < poses.size(); i++) {
      newPoses[i] = poses.get(i);
    }
    return newPoses;
  }

  public VisionIOPhotonVisionCoral getCamera(int index) {
    return (VisionIOPhotonVisionCoral) io[index];
  }

  public void updateModels(Pose3d[] poses) {
    VisionTargetSim[] visionTargets = new VisionTargetSim[poses.length];
    for (int i = 0; i < poses.length; i++) {
      Pose3d targetPose =
          poses[i].plus(new Transform3d(0, 0, Units.inchesToMeters(4.5 / 2), new Rotation3d()));
      visionTargets[i] = new VisionTargetSim(targetPose, targetModel);
    }
    for (int v = 0; v < io.length; v++) {
      io[v].updateModels(visionTargets);
    }
  }

  @AutoLogOutput
  public boolean isValid() {
    for (int i = 0; i < io.length; i++) {
      // If there's more than 2 coral that a cam can see
      if (inputs[i].coralPoseEstimates.length > 2) {
        return false;
      }
    }
    return true;
  }
}
