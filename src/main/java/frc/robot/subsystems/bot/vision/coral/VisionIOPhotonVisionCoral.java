// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.bot.vision.coral;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers.CoralObservation;
import frc.robot.subsystems.bot.drive.Drive.VisionParameters;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVisionCoral implements VisionIOCoral {
  final PhotonCamera camera;
  private Transform3d robotToCamera;
  private Supplier<VisionParameters> visionParams;
  List<PhotonPipelineResult> cameraResults;
  PhotonPipelineResult latestResult;
  List<PhotonTrackedTarget> cameraTargets;

  public Pose3d[] coralPoses = new Pose3d[] {};
  public Pose3d[] acceptedCoralPoses = new Pose3d[] {};
  public Pose3d[] rejectedCoralPoses = new Pose3d[] {};

  public VisionIOPhotonVisionCoral( // Creating class
      String cameraName, Transform3d robotToCamera, Supplier<VisionParameters> visionParams) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.visionParams = visionParams;
    Logger.recordOutput(
        "TempCoralsBCFun",
        new Pose3d[] {
          new Pose3d(0, 0, 0, new Rotation3d(0, 0, Units.degreesToRadians(90))),
          new Pose3d(1, 1, 0, new Rotation3d(0, 0, 0))
        });
  }

  @Override
  public void updateInputs(VisionIOCoralInputs inputs) {
    inputs.connected = camera.isConnected();
    CoralObservation observation = getEstimatedCoralPoses();
    inputs.coralPoseEstimates = observation.coralEstimates();
    inputs.acceptedCoralPoseEstimates = observation.acceptedCoralEstimates();
    inputs.rejectedCoralPoseEstimates = observation.rejectedCoralEstimates();
  }

  private CoralObservation getEstimatedCoralPoses() {
    updateResults();
    if (cameraResults.isEmpty()) return new CoralObservation();

    PhotonPipelineResult latestResult = cameraResults.get(cameraResults.size() - 1);
    if (!latestResult.hasTargets()) {
      return new CoralObservation();
    }

    coralPoses = new Pose3d[latestResult.getTargets().size()];
    acceptedCoralPoses = new Pose3d[0];
    rejectedCoralPoses = new Pose3d[0];

    if (latestResult.hasTargets()) {
      for (int i = 0; i < latestResult.getTargets().size(); i++) {
        var target = latestResult.targets.get(i);
        var coralCorners = target.getDetectedCorners();

        double fovx = 77.3 / 2;
        double fovy = 61.9 / 2;

        double maxx =
            Double.max(
                coralCorners.get(0).x, Double.max(coralCorners.get(1).x, coralCorners.get(2).x));
        double minx =
            Double.min(
                coralCorners.get(0).x, Double.min(coralCorners.get(1).x, coralCorners.get(2).x));
        double maxy =
            Double.max(
                coralCorners.get(0).y, Double.max(coralCorners.get(1).y, coralCorners.get(2).y));
        double miny =
            Double.min(
                coralCorners.get(0).y, Double.min(coralCorners.get(1).y, coralCorners.get(2).y));

        double cenx = (maxx + minx) / 2;
        double ceny = (maxy + miny) / 2;

        double xdeg = ((960 / 2) - cenx) / (960 / 2) * fovx;
        double ydeg = ((720 / 2) - ceny) / (720 / 2) * fovy;

        Pose3d rotatedPose =
            new Pose3d(visionParams.get().robotPose())
                .plus(robotToCamera)
                .plus(
                    new Transform3d(
                        0,
                        0,
                        0,
                        new Rotation3d(
                            0, Units.degreesToRadians(-ydeg), Units.degreesToRadians(xdeg))));

        Pose3d prismPose = findPoseAlongLineAtHeight(rotatedPose);

        double deg0 = 11.875 / 4.5;

        double xtot = maxx - minx;
        double ytot = maxy - miny;

        double deg = xtot / ytot;

        // verry verry rough estimate
        double yaw;
        if (deg <= 1) {
          yaw = 0;
        } else if (deg >= deg0) {
          yaw = 90;
        } else {
          yaw = 90 * deg / deg0;
        }

        prismPose =
            new Pose3d(
                prismPose.getTranslation(), new Rotation3d(0, 0, Units.degreesToRadians(90 - yaw)));

        if (prismPose
                .getTranslation()
                .toTranslation2d()
                .getDistance(visionParams.get().robotPose().getTranslation())
            < 5) {
          acceptedCoralPoses = addPose(acceptedCoralPoses, prismPose);
        } else {
          rejectedCoralPoses = addPose(rejectedCoralPoses, prismPose);
        }

        coralPoses[i] = prismPose;
      }
      return buildPoseObservation(coralPoses, acceptedCoralPoses, rejectedCoralPoses);
    }
    return new CoralObservation();
  }

  private Pose3d[] addPose(Pose3d[] pose3ds, Pose3d newPose) {
    Pose3d[] newPoses = new Pose3d[pose3ds.length + 1];
    for (int i = 0; i < pose3ds.length; i++) {
      newPoses[i] = pose3ds[i];
    }
    newPoses[pose3ds.length] = newPose;
    return newPoses;
  }

  private CoralObservation buildPoseObservation(
      Pose3d[] coralPoses, Pose3d[] acceptedCoralPoses, Pose3d[] rejectedCoralPoses) {
    return new CoralObservation(coralPoses, acceptedCoralPoses, rejectedCoralPoses);
  }

  private void updateResults() {
    cameraResults = camera.getAllUnreadResults();
    if (!cameraResults.isEmpty()) {
      latestResult = cameraResults.get(cameraResults.size() - 1);
    } else {
      latestResult = new PhotonPipelineResult();
    }
    if (latestResult.hasTargets()) {
      cameraTargets = latestResult.targets;
    } else {
      cameraTargets = new ArrayList<>();
    }
  }

  public static Pose3d findPoseAlongLineAtHeight(Pose3d originalPose) {
    double targetHeightMeters = Units.inchesToMeters(2.25);

    Translation3d currentTranslation = originalPose.getTranslation();
    Rotation3d currentRotation = originalPose.getRotation();

    double x0 = currentTranslation.getX();
    double y0 = currentTranslation.getY();
    double z0 = currentTranslation.getZ();

    Translation3d directionVector = new Translation3d(1, 0, 0).rotateBy(currentRotation);

    double dx = directionVector.getX();
    double dy = directionVector.getY();
    double dz = directionVector.getZ();

    if (Math.abs(dz) < 1e-9) {
      if (Math.abs(z0 - targetHeightMeters) > 1e-9) {
        System.err.println(
            "Cannot reach target height: Pose is horizontal and not at target height.");
        return null;
      } else {
        System.err.println("Already at height");
        return originalPose;
      }
    }

    double t = (targetHeightMeters - z0) / dz;

    double newX = x0 + t * dx;
    double newY = y0 + t * dy;
    double newZ = targetHeightMeters;

    Translation3d newTranslation = new Translation3d(newX, newY, newZ);

    return new Pose3d(newTranslation, currentRotation);
  }
}
