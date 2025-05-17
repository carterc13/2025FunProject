package frc.robot;

public class State {
    private static GamePieceStates gamePieceState = GamePieceStates.NONE;
    private static DriveStates driveState = DriveStates.IDLE;
    private static GameStates gameState = GameStates.TELEOP;

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

    public static void setGamePieceState(GamePieceStates state) {
        gamePieceState = state;
    }

    public static void setDriveState(DriveStates state) {
        driveState = state;
    }

    public static void setGameState(GameStates state) {
        gameState = state;
    }
}
