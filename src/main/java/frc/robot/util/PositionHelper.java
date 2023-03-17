package frc.robot.util;

import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;

public final class PositionHelper {
    
    public static double[] getDouble(Position position, GamePieceMode mode) {
        switch(position) {
            case Ground:
                return getGround(mode);
            case Mid:
                return getMid(mode);
            case High:
                return getHigh(mode);
            case Shelf:
                return getShelf(mode);
            default:
                return ArmPositions.stow;
        }
    }

    private static double[] getGround(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.intakeCubeGround;
        if (mode == GamePieceMode.ConeDown) return ArmPositions.intakeConeDownGround;
        if (mode == GamePieceMode.ConeUp) return ArmPositions.intakeConeUpGround;
        return null;
    }

    private static double[] getMid(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.placeCubeMid;
        if (mode == GamePieceMode.ConeDown) return ArmPositions.placeConeDownMid;
        if (mode == GamePieceMode.ConeUp) return ArmPositions.placeConeUpMid;
        return null;
    }
    
    private static double[] getHigh(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.placeCubeHigh;
        if (mode == GamePieceMode.ConeDown) return ArmPositions.placeConeDownHigh;
        if (mode == GamePieceMode.ConeUp) return ArmPositions.placeConeUpHigh;
        return null;
    }

    private static double[] getShelf(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.intakeCubeShelf;
        if (mode == GamePieceMode.ConeDown) return ArmPositions.intakeConeDownShelf;
        if (mode == GamePieceMode.ConeUp) return ArmPositions.intakeConeUpShelf;
        return null;
    }
}
