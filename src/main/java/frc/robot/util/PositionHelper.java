package frc.robot.util;

import frc.robot.RobotState;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPosition;

public final class PositionHelper {
    
    public static ArmPosition getPosition(Position position, GamePieceMode mode) {
        switch(position) {
            case Ground:
                return getGround(mode);
            case Mid:
                return getMid(mode);
            case High:
                return getHigh(mode);
            case Shelf:
                return ArmPositions.intakeConeUpShelf;
            default:
                return ArmPositions.stow;
        }
    }

    private static ArmPosition getGround(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.intakeCubeGround;
        if (mode == GamePieceMode.ConeDown) {
            return ArmPositions.intakeConeDownGround;
        }
        if (mode == GamePieceMode.ConeUp)  {
            return ArmPositions.intakeConeUpGround;
        }
        return null;
    }

    private static ArmPosition getMid(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.placeCubeMid;
        if (mode == GamePieceMode.ConeDown) return ArmPositions.placeConeDownMid;
        if (mode == GamePieceMode.ConeUp) return ArmPositions.placeConeUpMid;
        return null;
    }
    
    private static ArmPosition getHigh(GamePieceMode mode) {
        if (mode == GamePieceMode.Cube) return ArmPositions.placeCubeHigh;
        if (mode == GamePieceMode.ConeDown) return ArmPositions.placeConeDownHigh;
        if (mode == GamePieceMode.ConeUp) return ArmPositions.placeConeUpHigh;
        return null;
    }
}
