// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.bot.drive.Drive.ReefPositions;
import frc.robot.subsystems.bot.elevator.Elevator.ElevatorPosition;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SimCoral {
  private static ArrayList<Pose3d> poses = new ArrayList<>();
  private static Pose2d l1 = new Pose2d();
  private static Pose2d l2 = new Pose2d();
  private static Pose2d r1 = new Pose2d();
  private static Pose2d r2 = new Pose2d();

  public static void start() {
    PoseComputer.setRedAlliance();
    poses.add(0, new Pose3d());
    poses.add(1, new Pose3d());
    Logger.recordOutput("Coral", poses.toArray(new Pose3d[poses.size()]));
    SmartDashboard.putNumber("Coral/x", 0);
    SmartDashboard.putNumber("Coral/y", 0);
    SmartDashboard.putNumber("Coral/z", 0);
    SmartDashboard.putNumber("Coral/roll", 0);
    SmartDashboard.putNumber("Coral/pitch", 0);
    SmartDashboard.putNumber("Coral/yaw", 0);
  }

  public static void loggingPeriodic(Pose2d pose) {
    l1 =
        getLeftPose()
            .plus(
                new Transform2d(
                    0, Units.inchesToMeters(-32), new Rotation2d(Units.degreesToRadians(-90))));
    l2 =
        getLeftPose()
            .plus(
                new Transform2d(
                    0, Units.inchesToMeters(32), new Rotation2d(Units.degreesToRadians(90))));
    r1 =
        getRightPose()
            .plus(
                new Transform2d(
                    0, Units.inchesToMeters(-32), new Rotation2d(Units.degreesToRadians(-90))));
    r2 =
        getRightPose()
            .plus(
                new Transform2d(
                    0, Units.inchesToMeters(32), new Rotation2d(Units.degreesToRadians(90))));

    Logger.recordOutput("Possible Poses", new Pose2d[] {l1, l2, r1, r2});
    Logger.recordOutput("Possible Tragectory 1", new Pose2d[] {l1, pose});
    Logger.recordOutput("Possible Tragectory 2", new Pose2d[] {l2, pose});
    Logger.recordOutput("Possible Tragectory 3", new Pose2d[] {r1, pose});
    Logger.recordOutput("Possible Tragectory 4", new Pose2d[] {r2, pose});
  }

  public static void setPose(int index, Pose3d pose) {
    poses.set(index, pose);
    Logger.recordOutput("Coral", poses.toArray(new Pose3d[poses.size()]));
  }

  public static Pose2d getLeftPose() {
    return poses.get(1).toPose2d();
  }

  public static Pose2d getRightPose() {
    return poses.get(0).toPose2d();
  }

  public static Pose3d[] getGroundCoral() {
    Pose3d[] groundPoses = new Pose3d[2];
    groundPoses[0] = poses.get(0);
    groundPoses[1] = poses.get(1);
    return groundPoses;
  }

  public static void placeCoral(ElevatorPosition level, ReefPositions position) {
    spots spot = getSpot(position, level);
    if (spot != null && spot != spots.Level1) {
      poses.add(getSpot(position, level).pose);
    } else if (spot == spots.Level1) {
      // do smth
    }
    Logger.recordOutput("Coral", poses.toArray(new Pose3d[poses.size()]));
  }

  public static void tempCoral() {
    Logger.recordOutput(
        "TMP Coral",
        new Pose3d(
            SmartDashboard.getNumber("Coral/x", 0),
            SmartDashboard.getNumber("Coral/y", 0),
            SmartDashboard.getNumber("Coral/z", 0),
            new Rotation3d(
                Units.degreesToRadians(SmartDashboard.getNumber("Coral/roll", 0)),
                Units.degreesToRadians(SmartDashboard.getNumber("Coral/pitch", 0)),
                Units.degreesToRadians(SmartDashboard.getNumber("Coral/yaw", 0)))));
    Logger.recordOutput(
        "TMP Pose",
        new Pose3d(
                SmartDashboard.getNumber("Coral/x", 0),
                SmartDashboard.getNumber("Coral/y", 0),
                SmartDashboard.getNumber("Coral/z", 0),
                new Rotation3d(
                    Units.degreesToRadians(SmartDashboard.getNumber("Coral/roll", 0)),
                    Units.degreesToRadians(SmartDashboard.getNumber("Coral/pitch", 0)),
                    Units.degreesToRadians(SmartDashboard.getNumber("Coral/yaw", 0))))
            .toPose2d()
            .plus(
                new Transform2d(
                    0, Units.inchesToMeters(-32), new Rotation2d(Units.degreesToRadians(90)))));
  }

  public static Command DropR() {
    return new DropR();
  }

  public static Command DropL() {
    return new DropL();
  }

  public static enum spots {
    Level1(),
    A2(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.78,
                4.189,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(0.0))))),
    B2(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.78,
                3.8625,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(0.0))))),
    C2(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.99,
                3.49,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(60.0))))),
    D2(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.275,
                3.325,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(60.0))))),
    E2(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.705,
                3.325,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(120.0))))),
    F2(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.99,
                3.49,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(120.0))))),
    G2(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.2,
                3.8625,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(180.0))))),
    H2(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.2,
                4.189,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(180.0))))),
    I2(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.99,
                4.56,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(240.0))))),
    J2(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.705,
                4.73,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(240.0))))),
    K2(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.275,
                4.73,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(300.0))))),
    L2(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.99,
                4.56,
                0.725,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(300.0))))),
    A3(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.78,
                4.189,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(0.0))))),
    B3(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.78,
                3.8625,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(0.0))))),
    C3(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.99,
                3.49,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(60.0))))),
    D3(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.275,
                3.325,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(60.0))))),
    E3(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.705,
                3.325,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(120.0))))),
    F3(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.99,
                3.49,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(120.0))))),
    G3(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.2,
                3.8625,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(180.0))))),
    H3(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.2,
                4.189,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(180.0))))),
    I3(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.99,
                4.56,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(240.0))))),
    J3(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.705,
                4.73,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(240.0))))),
    K3(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.275,
                4.73,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(300.0))))),
    L3(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.99,
                4.56,
                1.125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(30.0),
                    Units.degreesToRadians(300.0))))),
    A4(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.71,
                4.198,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(0.0))))),
    B4(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.71,
                3.8625,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(0.0))))),
    C4(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.96,
                3.435,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(60.0))))),
    D4(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.245,
                3.2775,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(60.0))))),
    E4(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.725,
                3.2775,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(120.0))))),
    F4(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.022,
                3.435,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(120.0))))),
    G4(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.265,
                3.8625,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(180.0))))),
    H4(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.265,
                4.189,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(180.0))))),
    I4(
        AllianceFlipUtil.apply(
            new Pose3d(
                5.022,
                4.625,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(240.0))))),
    J4(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.7325,
                4.785,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(240.0))))),
    K4(
        AllianceFlipUtil.apply(
            new Pose3d(
                4.245,
                4.785,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(300.0))))),
    L4(
        AllianceFlipUtil.apply(
            new Pose3d(
                3.96,
                4.625,
                1.7125,
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(75.0),
                    Units.degreesToRadians(300.0)))));

    private final Pose3d pose;

    spots(Pose3d pose) {
      this.pose = pose;
    }

    spots() {
      this.pose = new Pose3d();
    }
  }

  public static spots getSpot(ReefPositions target, ElevatorPosition level) {
    if (target == ReefPositions.A) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.A2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.A3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.A4;
      }
    }
    if (target == ReefPositions.B) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.B2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.B3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.B4;
      }
    }
    if (target == ReefPositions.C) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.C2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.C3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.C4;
      }
    }
    if (target == ReefPositions.D) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.D2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.D3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.D4;
      }
    }
    if (target == ReefPositions.E) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.E2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.E3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.E4;
      }
    }
    if (target == ReefPositions.F) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.F2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.F3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.F4;
      }
    }
    if (target == ReefPositions.G) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.G2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.G3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.G4;
      }
    }
    if (target == ReefPositions.H) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.H2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.H3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.H4;
      }
    }
    if (target == ReefPositions.I) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.I2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.I3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.I4;
      }
    }
    if (target == ReefPositions.J) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.J2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.J3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.J4;
      }
    }
    if (target == ReefPositions.K) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.K2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.K3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.K4;
      }
    }
    if (target == ReefPositions.L) {
      if (level == ElevatorPosition.L1) {
        return spots.Level1;
      }
      if (level == ElevatorPosition.L2) {
        return spots.L2;
      }
      if (level == ElevatorPosition.L3) {
        return spots.L3;
      }
      if (level == ElevatorPosition.L4) {
        return spots.L4;
      }
    }
    return null;
  }
}
