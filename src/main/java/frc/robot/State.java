package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.FieldConstants;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class State {
  private static GamePieceStates gamePieceState = GamePieceStates.NONE;
  private static DriveStates driveState = DriveStates.IDLE;
  private static GameStates gameState = GameStates.TELEOP;
  private static ReefPositions reefPosition = ReefPositions.A;
  private static boolean isRightSource = false;
  private static Pose3d nextCoralPose = new Pose3d();
  private static ArrayList<AllReefLocations> history = new ArrayList<>();
  private static final double xOffset = 24;
  private static final double yOffset = 7;
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

  public static enum GamePieceStates {
    NONE(),
    CORAL(),
    ALGAE();
  }

  public static enum DriveStates {
    IDLE(),
    ALIGNREEF(),
    AUTOTOCORAL(),
    AUTOTOREEF(),
    AUTOINTAKE(),
    SWICHREEFSPOTS(),
    INTAKEALGAE(),
    REEFTOBARGE();
  }

  public static enum GameStates {
    AUTO(),
    TELEOP(),
    ENDGAME();
  }

  public static enum ReefPositions {
    A(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(-yOffset))),
    B(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(yOffset))),
    C(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(-yOffset))),
    D(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(yOffset))),
    E(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(-yOffset))),
    F(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(yOffset))),
    G(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(-yOffset))),
    H(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(yOffset))),
    I(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(-yOffset))),
    J(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(yOffset))),
    K(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(-yOffset))),
    L(new Translation2d(Units.inchesToMeters(xOffset), Units.inchesToMeters(yOffset)));

    Translation2d translation;

    ReefPositions(Translation2d translation) {
      this.translation = translation;
    }

    public Transform3d getPose(int tag) {
      return new Transform3d(
              FieldConstants.aprilTags.getTagPose(tag).get().getTranslation(),
              FieldConstants.aprilTags.getTagPose(tag).get().getRotation())
          .plus(
              new Transform3d(
                  new Translation3d(
                      this.translation.getX(),
                      this.translation.getY(),
                      -FieldConstants.aprilTags.getTagPose(tag).get().getZ()),
                  new Rotation3d(0, 0, -Math.PI)));
    }
  }

  public static int getTagForTarget(ReefPositions target) {
    return switch (target) {
      case A, B -> !redalliance ? 7 : 18;
      case C, D -> !redalliance ? 8 : 17;
      case E, F -> !redalliance ? 9 : 22;
      case G, H -> !redalliance ? 10 : 21;
      case I, J -> !redalliance ? 11 : 20;
      case K, L -> !redalliance ? 6 : 19;
    };
  }

  public static enum AllReefLocations {
    Level1(),
    A2(),
    B2(),
    C2(),
    D2(),
    E2(),
    F2(),
    G2(),
    H2(),
    I2(),
    J2(),
    K2(),
    L2(),
    A3(),
    B3(),
    C3(),
    D3(),
    E3(),
    F3(),
    G3(),
    H3(),
    I3(),
    J3(),
    K3(),
    L3(),
    A4(),
    B4(),
    C4(),
    D4(),
    E4(),
    F4(),
    G4(),
    H4(),
    I4(),
    J4(),
    K4(),
    L4();
  }

  public static void addToHistory(AllReefLocations location) {
    history.add(location);
  }

  public static void setGamePieceState(GamePieceStates state) {
    gamePieceState = state;
  }

  public static GamePieceStates getGamePieceState() {
    return gamePieceState;
  }

  public static void setDriveState(DriveStates state) {
    driveState = state;
    Logger.recordOutput("Drive State", state.toString());
  }

  public static DriveStates getDriveState() {
    return driveState;
  }

  public static ReefPositions getReefPosition() {
    return reefPosition;
  }

  public static void setReefPosition(ReefPositions newPosition) {
    reefPosition = newPosition;
  }

  public static boolean isRightSource() {
    return isRightSource;
  }

  public static void setRightSource(boolean is) {
    isRightSource = is;
  }

  public static void setNextCoralPose(Pose3d pose) {
    nextCoralPose = pose;
  }

  public static Pose3d getNextCoralPose() {
    return nextCoralPose;
  }

  public static boolean isDriveIdle() {
    return getDriveState() == DriveStates.IDLE;
  }

  public static void setGameState(GameStates state) {
    gameState = state;
  }
}
