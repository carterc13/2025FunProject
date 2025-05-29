// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bot;

import java.util.ArrayList;

import frc.robot.subsystems.bot.drive.Drive.ReefPositions;
import frc.robot.subsystems.bot.elevator.Elevator.ElevatorPosition;

/** Add your docs here. */
public class Memory {
    private ArrayList<spots> mem = new ArrayList<spots>();
    private spots currentTarget = spots.A4;

    public Memory() {}

    public void place(ReefPositions target, ElevatorPosition level) {
        mem.add(getSpot(target, level));
    }

    public spots getCurrentTarget() {
        return currentTarget;
    }

    public enum spots {
    Level1(37),
    A2(25, 10),
    A3(13, 11),
    A4(1, 12),
    B2(26, 7),
    B3(14, 8),
    B4(2, 9),
    C2(27, 1),
    C3(15, 3),
    C4(3, 5),
    D2(28, 2),
    D3(16, 4),
    D4(4, 6),
    E2(29),
    E3(17),
    E4(5),
    F2(30),
    F3(18),
    F4(6),
    G2(31),
    G3(19),
    G4(7),
    H2(32),
    H3(20),
    H4(8),
    I2(33),
    I3(21),
    I4(9),
    J2(34),
    J3(22),
    J4(10),
    K2(35, 14),
    K3(23, 16),
    K4(11, 18),
    L2(36, 13),
    L3(24, 15),
    L4(12, 17);

    private final int branchID;
    private final int cyclesCycleNum;
    private final int pointsCycleNum;
    private final int rpCycleNum;
    private final boolean rightHalf;
    private final boolean frontHalf;

    spots(int branchID, int cyclesCycleNum, int pointsCycleNum, int rpCycleNum, boolean rightHalf, boolean frontHalf) {
        this.branchID = branchID;
        this.cyclesCycleNum = cyclesCycleNum;
        this.pointsCycleNum = pointsCycleNum;
        this.rpCycleNum = rpCycleNum;
        this.rightHalf = rightHalf;
        this.frontHalf = frontHalf;
    }
  }

  public spots getSpot(ReefPositions target, ElevatorPosition level) {
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
