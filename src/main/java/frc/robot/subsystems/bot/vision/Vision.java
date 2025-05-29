// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.bot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseObservation;
import frc.robot.subsystems.bot.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.bot.vision.VisionUtil.VisionData;
import frc.robot.subsystems.bot.vision.VisionUtil.VisionMeasurement;
import frc.robot.subsystems.bot.vision.VisionUtil.VisionMode;
import frc.robot.utils.FieldConstants;
import java.util.*;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that handles vision processing from multiple cameras using AprilTags. Processes data
 * from MegaTag1 and MegaTag2 vision systems, validates measurements, and provides filtered vision
 * data to the robot's pose estimator.
 */
public class Vision extends SubsystemBase {

  private static final VisionMode MODE = VisionMode.MA;
  private static final String VISION_PATH = "Vision/Camera";

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final double flagAngle =
      Math.toRadians(
          2.0); // Pitch and roll of a reading before a camera needs to be flagged. IN RADIANS FOR
  // GODS SAKE

  private boolean[] rejectCamera = {false, false, false, false};

  /**
   * Lists to store vision measurements and poses. These are maintained at the class level to allow
   * for logging during both real and simulation operation.
   */
  private List<VisionMeasurement> measurements = new ArrayList<>();

  private List<Pose3d> tagPoses = new ArrayList<>();
  private List<Pose3d> acceptedTagPoses = new ArrayList<>();
  private List<Pose3d> rejectedTagPoses = new ArrayList<>();
  private List<Pose3d> robotPoses = new ArrayList<>();
  private List<Pose3d> acceptedPoses = new ArrayList<>();
  private List<Pose3d> rejectedPoses = new ArrayList<>();

  /**
   * Creates a new Vision subsystem.
   *
   * @param consumer Callback interface for processed vision measurements
   * @param io Array of VisionIO interfaces for each camera
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    System.out.println("[Init] Creating Vision");
    this.consumer = consumer;
    this.io = io;

    // Initialize input arrays for each camera
    inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnection alerts for each camera
    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(String.format("Vision camera %d is disconnected.", i), AlertType.kWarning);
    }

    SmartDashboard.putBoolean("Disable Front Left Cam", false);
    SmartDashboard.putBoolean("Disable Front Right Cam", false);
    SmartDashboard.putBoolean("Disable Back Left Cam", false);
    SmartDashboard.putBoolean("Disable Back Right Cam", false);
  }

  @Override
  public void periodic() {
    // Update inputs and check connection status for each camera
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
      Logger.processInputs(VISION_PATH + i, inputs[i]);
    }

    // Process vision data and send to consumer
    VisionData visionData = processAllCameras();
    logSummary(visionData);
    consumer.accept(sortMeasurements(visionData.measurements()));

    for (int i = 0; i < io.length; i++) {
      VisionIOPhotonVision currentCamera = getCamera(i); // Grabs current camera
      if (currentCamera.hasTargets()) {
        Rotation3d rotationReading =
            currentCamera.lastAcceptedPose.getRotation(); // Grab rotation from reading for skew
        Logger.recordOutput(
            "VisionDebugging/Camera " + i + " pose",
            currentCamera.lastAcceptedPose); // Log entire pose from camera
        boolean flagged =
            (rotationReading.getX() <= -flagAngle || rotationReading.getX() >= flagAngle)
                || (rotationReading.getY() <= -flagAngle
                    || rotationReading.getY() >= flagAngle); // Logic to flag camera
        rejectCamera[i] = flagged; // Here is where the camera is rejected if skewed
        currentCamera.flag(flagged); // This flags the camera in the camera class
      }
      Logger.recordOutput("VisionDebugging/Camera " + i + " flagged", getCamera(i).flagged);
    }
  }

  public VisionIOPhotonVision getCamera(int index) {
    return (VisionIOPhotonVision) io[index];
  }

  /**
   * Calculates the robot's offset from the specified tag using only the front 2 cameras
   *
   * @param id The requested AprilTag's id
   * @param desiredOffset The desired offset of the robot relative to the tag
   * @return Transform3d that represents the robots position relative to the offset
   */
  public Transform3d calculateOffset(int id, Translation2d desiredOffset) {
    Transform3d leftCamToTag = getCamera(0).getRobotToTargetOffset(id);
    Transform3d rightCamToTag = getCamera(1).getRobotToTargetOffset(id);

    try {
      Logger.recordOutput(
          "VisionDebugging/target position",
          new Transform3d(
                  FieldConstants.aprilTags.getTagPose(id).get().getTranslation(),
                  FieldConstants.aprilTags.getTagPose(id).get().getRotation())
              .plus(
                  new Transform3d(
                      new Translation3d(
                          desiredOffset.getX(),
                          desiredOffset.getY(),
                          -FieldConstants.aprilTags.getTagPose(id).get().getZ()),
                      new Rotation3d(-Math.PI, 0, 0))));
      Logger.recordOutput(
          "VisionDebugging/left cam based target position",
          leftCamToTag
              .plus(
                  new Transform3d(
                      new Translation3d(desiredOffset.getX(), desiredOffset.getY(), 0),
                      new Rotation3d()))
              .inverse());
      Logger.recordOutput(
          "VisionDebugging/right cam based target position",
          rightCamToTag
              .plus(
                  new Transform3d(
                      new Translation3d(desiredOffset.getX(), desiredOffset.getY(), 0),
                      new Rotation3d()))
              .inverse());
      Logger.recordOutput(
          "VisionDebugging/Left Cam Offset",
          getCamera(0).getTarget(id).bestCameraToTarget.getTranslation().toVector().getData());
      Logger.recordOutput(
          "VisionDebugging/Right Cam Offset",
          getCamera(1).getTarget(id).bestCameraToTarget.getTranslation().toVector().getData());
    } catch (Exception e) {

    }

    // Return the non-zero offset, or kZero if both are zero
    var result =
        !leftCamToTag.equals(Transform3d.kZero)
            ? leftCamToTag
            : (!rightCamToTag.equals(Transform3d.kZero)
                ? rightCamToTag
                : new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()));

    Logger.recordOutput("X to Tag", result.getX());
    Logger.recordOutput("Y to Tag", result.getY());

    return result;
  }

  /**
   * Checks the front 2 cameras to see if there is a result with the fiducial ID requested
   *
   * @param id Requested tag's fiducial ID
   * @return If one of the front 2 cameras have the requested target, returns true
   */
  public boolean containsRequestedTarget(int id) {
    return getCamera(0).hasTarget(id) || getCamera(1).hasTarget(id);
  }

  /**
   * Calculates the distance to the requested tag; Make sure to check if the tag exists before you
   * call this command
   *
   * @param tagID Requested tag's ID
   * @return Distance from the robot's center to the target found
   */
  public double getDistanceToTag(int tagID) {
    return Math.sqrt(
        Math.pow(calculateOffset(tagID, new Translation2d()).getX(), 2)
            + Math.pow(calculateOffset(tagID, new Translation2d()).getY(), 2));
  }

  /**
   * //////// WIP //////// TODO change the privacy once done
   *
   * @param index
   */
  private void recalibrateCamera(int index) {
    return;
  }

  /**
   * Recalibrates either the front left or the right camera based on the parameters given.
   *
   * @param leftCamera Boolean to tell whether the camera being calibrated is the left or the right
   *     camera
   * @param tagID Reference tag ID that BOTH of the tags can see; if they don't, nothing will happen
   */
  public void recalibrateFrontCamera(boolean leftCamera, int tagID) {
    if (!(getCamera(0).hasTarget(tagID) && getCamera(1).hasTarget(tagID))) {
      return;
    }
    Transform3d uncalibratedReading = getCamera(leftCamera ? 0 : 1).getCameraToTag(tagID);
    Transform3d referenceReading = getCamera(leftCamera ? 1 : 0).getCameraToTag(tagID);
    Logger.recordOutput("VisionDebugging/Uncalibrated Reading Left", uncalibratedReading);
    Logger.recordOutput("VisionDebugging/Reference Reading", referenceReading);
    getCamera(leftCamera ? 0 : 1)
        .setStdDev(
            getCamera(leftCamera ? 1 : 0)
                .getStdDev()
                .plus(referenceReading)
                .plus(uncalibratedReading.inverse()));
  }

  /**
   * Only use this method if you are completely sure there is a camera with the target or else it
   * will not work as expected.
   *
   * @param tagID The tag ID to find the camera for
   * @return The camera ID that has the target
   */
  public int getCameraIDWithTarget(int tagID) {
    for (int i = 0; i < io.length; i++) {
      if (getCamera(i).hasTarget(tagID)) {
        return i;
      }
    }
    return 0;
  }

  /**
   * Processes vision data from all cameras and combines the results.
   *
   * @return Combined VisionData from all cameras
   */
  private VisionData processAllCameras() {
    return Arrays.stream(inputs)
        .map(input -> processCamera(Arrays.asList(inputs).indexOf(input), input))
        .reduce(VisionData.empty(), VisionData::merge);
  }

  /**
   * Processes vision data from a single camera.
   *
   * @param cameraIndex Index of the camera being processed
   * @param input Input data from the camera
   * @return Processed VisionData for this camera
   */
  private VisionData processCamera(int cameraIndex, VisionIOInputs input) {
    rejectCamera[0] = SmartDashboard.getBoolean("Disable Front Left Cam", false);
    rejectCamera[1] = SmartDashboard.getBoolean("Disable Front Right Cam", false);
    rejectCamera[2] = SmartDashboard.getBoolean("Disable Back Left Cam", false);
    rejectCamera[3] = SmartDashboard.getBoolean("Disable Back Right Cam", false);

    if (rejectCamera[cameraIndex]) return VisionData.empty();

    PoseObservation[] poseObservations = {
      new PoseObservation(input.poseEstimateMT1, input.rawFiducialsMT1),
      new PoseObservation(input.poseEstimateMT2, input.rawFiducialsMT2)
    };

    return Arrays.stream(poseObservations)
        .filter(PoseObservation::isValid)
        .map(observation -> processObservation(cameraIndex, observation))
        .reduce(VisionData.empty(), VisionData::merge);
  }

  /**
   * Processes a single pose observation from a camera.
   *
   * @param cameraIndex Index of the camera that made the observation
   * @param observation The pose observation to process
   * @return Processed VisionData for this observation
   */
  private VisionData processObservation(int cameraIndex, PoseObservation observation) {
    // Clear previous data
    measurements.clear();
    tagPoses.clear();
    acceptedTagPoses.clear();
    rejectedTagPoses.clear();
    robotPoses.clear();
    acceptedPoses.clear();
    rejectedPoses.clear();

    if (Constants.currentMode == Constants.Mode.REAL) {
      return realObservation(cameraIndex, observation);
    } else {
      return simObservation(cameraIndex, observation);
    }
  }

  private VisionData realObservation(int cameraIndex, PoseObservation observation) {
    // Validate measurement against current vision mode criteria
    boolean acceptedVisionMeasurement = MODE.acceptVisionMeasurement(observation);

    // Add to appropriate accepted/rejected lists
    if (acceptedVisionMeasurement) {
      measurements.add(MODE.getVisionMeasurement(observation.poseEstimate()));
    }

    // Create and log vision data
    VisionData data =
        new VisionData(
            measurements,
            tagPoses,
            acceptedTagPoses,
            rejectedTagPoses,
            robotPoses,
            acceptedPoses,
            rejectedPoses);

    return data;
  }

  private VisionData simObservation(int cameraIndex, PoseObservation observation) {
    Pose3d robotPose = observation.poseEstimate().pose();
    robotPoses.add(robotPose);

    // Validate measurement against current vision mode criteria
    boolean acceptedVisionMeasurement = MODE.acceptVisionMeasurement(observation);

    // Process detected AprilTags
    for (var tag : observation.rawFiducials()) {
      FieldConstants.aprilTags
          .getTagPose(tag.id())
          .ifPresent(
              pose -> {
                tagPoses.add(pose);
                if (acceptedVisionMeasurement) {
                  acceptedTagPoses.add(pose);
                } else {
                  rejectedTagPoses.add(pose);
                }
              });
    }

    // Add to appropriate accepted/rejected lists
    if (acceptedVisionMeasurement) {
      measurements.add(MODE.getVisionMeasurement(observation.poseEstimate()));
      acceptedPoses.add(robotPose);
    } else {
      rejectedPoses.add(robotPose);
    }

    // Create and log vision data
    VisionData data =
        new VisionData(
            measurements,
            tagPoses,
            acceptedTagPoses,
            rejectedTagPoses,
            robotPoses,
            acceptedPoses,
            rejectedPoses);

    String mtType = observation.poseEstimate().isMegaTag2() ? "/MegaTag2" : "/MegaTag1";
    logCameraData(cameraIndex, mtType, data);

    return data;
  }

  /** Logs vision data for a specific camera and MegaTag type. */
  private void logCameraData(int cameraIndex, String mtType, VisionData data) {
    logPoses(VISION_PATH + cameraIndex + mtType, data);
  }

  /** Logs summary of all vision data. */
  private void logSummary(VisionData data) {
    logPoses("Vision/Summary", data);
  }

  /** Logs pose data to AdvantageKit. */
  private void logPoses(String basePath, VisionData data) {
    Logger.recordOutput(basePath + "/TagPoses", toPose3dArray(data.tagPoses()));
    Logger.recordOutput(basePath + "/TagPosesAccepted", toPose3dArray(data.acceptedTagPoses()));
    Logger.recordOutput(basePath + "/TagPosesRejected", toPose3dArray(data.rejectedTagPoses()));
    Logger.recordOutput(basePath + "/RobotPoses", toPose3dArray(data.robotPoses()));
    Logger.recordOutput(basePath + "/RobotPosesAccepted", toPose3dArray(data.acceptedPoses()));
    Logger.recordOutput(basePath + "/RobotPosesRejected", toPose3dArray(data.rejectedPoses()));
  }

  /** Converts a list of poses to an array. */
  private Pose3d[] toPose3dArray(List<Pose3d> poses) {
    return poses.toArray(new Pose3d[poses.size()]);
  }

  /** Sorts vision measurements by timestamp. */
  private List<VisionMeasurement> sortMeasurements(List<VisionMeasurement> measurements) {
    return measurements.stream()
        .sorted(
            (vm1, vm2) ->
                Double.compare(
                    vm1.poseEstimate().timestampSeconds(), vm2.poseEstimate().timestampSeconds()))
        .toList();
  }

  /** Functional interface for consuming processed vision measurements. */
  @FunctionalInterface
  public static interface VisionConsumer {
    void accept(List<VisionMeasurement> visionMeasurements);
  }
}
