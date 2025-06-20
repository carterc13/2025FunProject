// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.bot.vision.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.bot.drive.Drive.VisionParameters;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class VisionIOPhotonVisionSIMCoral extends VisionIOPhotonVisionCoral {
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;
  private Supplier<VisionParameters> visionParams;

  /**
   * Constructs a PhotonVision SIM object with the specified camera name and camera position.
   *
   * @param cameraName the name of the camera
   * @param robotToCamera gets positon of camera to robot
   * @param poseSupplier current pose of robot
   */
  public VisionIOPhotonVisionSIMCoral(
      String cameraName, Transform3d robotToCamera, Supplier<VisionParameters> visionParams) {
    super(cameraName, robotToCamera, visionParams);

    if (visionSim == null) {
      visionSim = new VisionSystemSim("coral");
    }

    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProperties.setCalibError(0, 0); // 0.1, 0.10);
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableDrawWireframe(true);
    this.visionParams = visionParams;
  }

  @Override
  public void updateInputs(VisionIOCoralInputs inputs) {
    visionSim.update(visionParams.get().robotPose());
    super.updateInputs(inputs);
  }

  @Override
  public void updateModels(VisionTargetSim[] models) {
    visionSim.clearVisionTargets();
    for (int i = 0; i < models.length; i++) {
      // if (!visionSim.getVisionTargets().contains(models[i])) {
      visionSim.addVisionTargets("Coral" + i, models[i]);
      // }
    }
  }
}
