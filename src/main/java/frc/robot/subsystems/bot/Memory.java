// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.bot.Bot.BotType;
import frc.robot.subsystems.bot.drive.Drive.ReefPositions;
import frc.robot.subsystems.bot.elevator.Elevator.ElevatorPosition;
import java.util.ArrayList;

/** Add your docs here. */
public class Memory {
  // All reef positions that have coral on them
  private ArrayList<spots> mem2 = new ArrayList<spots>();
  // Current target branch
  private spots currentTarget = spots.A4;
  private final BotType type;

  public Memory(BotType type) {
    this.type = type;
  }

  /* Updates the reef position and level to now have coral on it */
  public void place(ReefPositions target, ElevatorPosition level) {
    mem2.add(getSpot(target, level));
  }

  /* Updates the reef position and level to now have coral on it */
  public void place() {
    mem2.add(currentTarget);
  }

  /* Updates the next target based on the bot pose */
  public void updateNextBest(Pose2d pose) {
    switch (type) {
      case CYCLES:
        double distance = 0;
        boolean good = false;
        // Cycles through all reef faces to make sure it has open spots and uses the one that is the
        // closest
        for (int i = 0; i < ReefPositions.values().length; i++) {
          if (pose.getTranslation()
                      .getDistance(
                          getPositionFromNum(i).getPose().getTranslation().toTranslation2d())
                  < distance
              && hasOpenSpots(getPositionFromNum(i))) {
            distance =
                pose.getTranslation()
                    .getDistance(
                        getPositionFromNum(i).getPose().getTranslation().toTranslation2d());
            currentTarget = getOpenSpot(getPositionFromNum(i));
          }
        }
        break;
      default:
        break;
    }
  }

  public spots getCurrentTarget() {
    return currentTarget;
  }

  private spots getOpenSpot(ReefPositions currentPosition) {
    switch (currentPosition) {
      case A:
        return !mem2.contains(spots.A2)
            ? spots.A2
            : !mem2.contains(spots.A3) ? spots.A3 : !mem2.contains(spots.A4) ? spots.A4 : null;
      case B:
        return !mem2.contains(spots.B2)
            ? spots.B2
            : !mem2.contains(spots.B3) ? spots.B3 : !mem2.contains(spots.B4) ? spots.B4 : null;
      case C:
        return !mem2.contains(spots.C2)
            ? spots.C2
            : !mem2.contains(spots.C3) ? spots.C3 : !mem2.contains(spots.C4) ? spots.C4 : null;
      case D:
        return !mem2.contains(spots.D2)
            ? spots.D2
            : !mem2.contains(spots.D3) ? spots.D3 : !mem2.contains(spots.D4) ? spots.D4 : null;
      case E:
        return !mem2.contains(spots.E2)
            ? spots.E2
            : !mem2.contains(spots.E3) ? spots.E3 : !mem2.contains(spots.E4) ? spots.E4 : null;
      case F:
        return !mem2.contains(spots.F2)
            ? spots.F2
            : !mem2.contains(spots.F3) ? spots.F3 : !mem2.contains(spots.F4) ? spots.F4 : null;
      case G:
        return !mem2.contains(spots.G2)
            ? spots.G2
            : !mem2.contains(spots.G3) ? spots.G3 : !mem2.contains(spots.G4) ? spots.G4 : null;
      case H:
        return !mem2.contains(spots.H2)
            ? spots.H2
            : !mem2.contains(spots.H3) ? spots.H3 : !mem2.contains(spots.H4) ? spots.H4 : null;
      case I:
        return !mem2.contains(spots.I2)
            ? spots.I2
            : !mem2.contains(spots.I3) ? spots.I3 : !mem2.contains(spots.I4) ? spots.I4 : null;
      case J:
        return !mem2.contains(spots.J2)
            ? spots.J2
            : !mem2.contains(spots.J3) ? spots.J3 : !mem2.contains(spots.J4) ? spots.J4 : null;
      case K:
        return !mem2.contains(spots.K2)
            ? spots.K2
            : !mem2.contains(spots.K3) ? spots.K3 : !mem2.contains(spots.K4) ? spots.K4 : null;
      case L:
        return !mem2.contains(spots.L2)
            ? spots.L2
            : !mem2.contains(spots.L3) ? spots.L3 : !mem2.contains(spots.L4) ? spots.L4 : null;
      default:
        return null;
    }
  }

  private boolean hasOpenSpots(ReefPositions position) {
    switch (position) {
      case A:
        return (mem2.contains(spots.A2) || mem2.contains(spots.A3) || mem2.contains(spots.A4));
      case B:
        return (mem2.contains(spots.B2) || mem2.contains(spots.B3) || mem2.contains(spots.B4));
      case C:
        return (mem2.contains(spots.C2) || mem2.contains(spots.C3) || mem2.contains(spots.C4));
      case D:
        return (mem2.contains(spots.D2) || mem2.contains(spots.D3) || mem2.contains(spots.D4));
      case E:
        return (mem2.contains(spots.E2) || mem2.contains(spots.E3) || mem2.contains(spots.E4));
      case F:
        return (mem2.contains(spots.F2) || mem2.contains(spots.F3) || mem2.contains(spots.F4));
      case G:
        return (mem2.contains(spots.G2) || mem2.contains(spots.G3) || mem2.contains(spots.G4));
      case H:
        return (mem2.contains(spots.H2) || mem2.contains(spots.H3) || mem2.contains(spots.H4));
      case I:
        return (mem2.contains(spots.I2) || mem2.contains(spots.I3) || mem2.contains(spots.I4));
      case J:
        return (mem2.contains(spots.J2) || mem2.contains(spots.J3) || mem2.contains(spots.J4));
      case K:
        return (mem2.contains(spots.K2) || mem2.contains(spots.K3) || mem2.contains(spots.K4));
      case L:
        return (mem2.contains(spots.L2) || mem2.contains(spots.L3) || mem2.contains(spots.L4));
      default:
        return false;
    }
  }

  private ReefPositions getPositionFromNum(int num) {
    if (num == 0) {
      return ReefPositions.A;
    } else if (num == 1) {
      return ReefPositions.B;
    } else if (num == 2) {
      return ReefPositions.C;
    } else if (num == 3) {
      return ReefPositions.D;
    } else if (num == 4) {
      return ReefPositions.E;
    } else if (num == 5) {
      return ReefPositions.F;
    } else if (num == 6) {
      return ReefPositions.G;
    } else if (num == 7) {
      return ReefPositions.H;
    } else if (num == 8) {
      return ReefPositions.I;
    } else if (num == 9) {
      return ReefPositions.J;
    } else if (num == 10) {
      return ReefPositions.K;
    } else {
      return ReefPositions.L;
    }
  }

  public enum spots {
    Level1(37, ElevatorPosition.L1),
    A2(25, ReefPositions.A, ElevatorPosition.L2),
    A3(13, ReefPositions.A, ElevatorPosition.L3),
    A4(1, ReefPositions.A, ElevatorPosition.L4),
    B2(26, ReefPositions.B, ElevatorPosition.L2),
    B3(14, ReefPositions.B, ElevatorPosition.L3),
    B4(2, ReefPositions.B, ElevatorPosition.L4),
    C2(27, ReefPositions.C, ElevatorPosition.L2),
    C3(15, ReefPositions.C, ElevatorPosition.L3),
    C4(3, ReefPositions.C, ElevatorPosition.L4),
    D2(28, ReefPositions.D, ElevatorPosition.L2),
    D3(16, ReefPositions.D, ElevatorPosition.L3),
    D4(4, ReefPositions.D, ElevatorPosition.L4),
    E2(29, ReefPositions.E, ElevatorPosition.L2),
    E3(17, ReefPositions.E, ElevatorPosition.L3),
    E4(5, ReefPositions.E, ElevatorPosition.L4),
    F2(30, ReefPositions.F, ElevatorPosition.L2),
    F3(18, ReefPositions.F, ElevatorPosition.L3),
    F4(6, ReefPositions.F, ElevatorPosition.L4),
    G2(31, ReefPositions.G, ElevatorPosition.L2),
    G3(19, ReefPositions.G, ElevatorPosition.L3),
    G4(7, ReefPositions.G, ElevatorPosition.L4),
    H2(32, ReefPositions.H, ElevatorPosition.L2),
    H3(20, ReefPositions.H, ElevatorPosition.L3),
    H4(8, ReefPositions.H, ElevatorPosition.L4),
    I2(33, ReefPositions.I, ElevatorPosition.L2),
    I3(21, ReefPositions.I, ElevatorPosition.L3),
    I4(9, ReefPositions.I, ElevatorPosition.L4),
    J2(34, ReefPositions.J, ElevatorPosition.L2),
    J3(22, ReefPositions.J, ElevatorPosition.L3),
    J4(10, ReefPositions.J, ElevatorPosition.L4),
    K2(35, ReefPositions.K, ElevatorPosition.L2),
    K3(23, ReefPositions.K, ElevatorPosition.L3),
    K4(11, ReefPositions.K, ElevatorPosition.L4),
    L2(36, ReefPositions.L, ElevatorPosition.L2),
    L3(24, ReefPositions.L, ElevatorPosition.L3),
    L4(12, ReefPositions.L, ElevatorPosition.L4);

    private final int branchID;
    private final ReefPositions positions;
    private final ElevatorPosition elePositions;

    public ReefPositions getReefPosition() {
      return this.positions;
    }

    public ElevatorPosition getElevatorPosition() {
      return this.elePositions;
    }

    spots(int branchID, ReefPositions positions, ElevatorPosition elePositions) {
      this.branchID = branchID;
      this.positions = positions;
      this.elePositions = elePositions;
    }

    spots(int branchID, ElevatorPosition elePositions) {
      this.branchID = branchID;
      this.positions = null;
      this.elePositions = elePositions;
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
