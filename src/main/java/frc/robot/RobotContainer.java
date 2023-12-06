// <https://www.youtube.com/watch?v=mXlsbBgrKKY>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.GamePieceMode;
import frc.robot.Constants.Position;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.auto.Align;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ActiveArmSide;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.PositionHelper;
import frc.robot.oi.ButtonMasher;
import frc.robot.oi.Thrustmaster;

public class RobotContainer {
    
    private final Drive drive = new Drive();
    // private final PivotSubsystem pivot = new PivotSubsystem();
    // private final TelescopeSubsystem telescope = new TelescopeSubsystem();
    // private final WristSubsystem wrist = new WristSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem(() -> arm.getArmSide());
    @SuppressWarnings("unused")
    private final Vision vision = new Vision();
    private final LEDManager ledManager = new LEDManager(() -> arm.getArmSide());

    private final Thrustmaster leftStick = new Thrustmaster(0);
    private final Thrustmaster rightStick = new Thrustmaster(1);
    private final ButtonMasher masher = new ButtonMasher(2);

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    private Command activeAutoCommand = null;
    private String activeAutoName = null;

    private DigitalInput brakeSwitch = new DigitalInput(DIOPorts.switch1);
    private DigitalInput ledsSwitch = new DigitalInput(DIOPorts.switch2);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        configureSubsystems();
        configureCompBindings();
        // configureTestBindings();
        configureAutos();
    }

    private void configureSubsystems() {
        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> -leftStick.getRawAxis(1),
            () -> -leftStick.getRawAxis(0),
            () -> -rightStick.getRawAxis(0),
            true
        ));
    }

    @SuppressWarnings("unused")
    private void configureTestBindings() {
        // masher.back()
        //     .onTrue(new MovePivot(pivot, 0.8))
        //     .onTrue(new MoveTelescope(telescope, pivot, 0.74))
        //     .onTrue(new MoveWrist(wrist, pivot, -0.43));

        // masher.start()
        //     .onTrue(new MovePivot(pivot, 0.69))
        //     .onTrue(new MoveTelescope(telescope, pivot, 0.78))
        //     .onTrue(new MoveWrist(wrist, pivot, -0.379));
        
        // masher.a()
        //     .onTrue(new InstantCommand(() -> RobotState.getInstance().invertBack()));

        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> -masher.getLeftY(),
            () -> -masher.getLeftX(),
            () -> -masher.getRightX(),
            true
        ));

        masher.a()
            .onTrue(new InstantCommand(() -> {
                drive.resetHeading(); 
                drive.setFieldToVehicle(
                    new Pose2d(
                        RobotState.getInstance().getFieldToVehicle().getTranslation(),
                        new Rotation2d(0)));
            }));
        
        // masher.b().onTrue(new TuneArmS(wrist));
    }   

    private void configureCompBindings() {
        // Reset heading
        rightStick.top()
            .onTrue(new InstantCommand(() -> {
                drive.resetHeading(); 
                drive.setFieldToVehicle(
                    new Pose2d(
                        RobotState.getInstance().getFieldToVehicle().getTranslation(),
                        new Rotation2d(0)));
            }));

        // Slow down drive
        leftStick.trigger()
            .onTrue(new InstantCommand(() -> drive.setBabyMode(true)))
            .onFalse(new InstantCommand(() -> drive.setBabyMode(false)));

        // Robot Relative Drive
        rightStick.trigger()
            .whileTrue(
                new DriveWithJoysticks(
                    drive,
                    () -> -leftStick.getRawAxis(1),
                    () -> -leftStick.getRawAxis(0),
                    () -> -rightStick.getRawAxis(0),
                    false
                )
            );

        // Snap robot heading to y-axis
        // leftStick.top().whileTrue(new SnapHeading(drive));

        // Auto-align
        leftStick.topLeft().whileTrue(new Align(drive, true));
        leftStick.topRight().whileTrue(new Align(drive, false));

        // Home
        rightStick.lowerButton(12).onTrue(new InstantCommand(arm::homeWrist)); // Home wrist
        rightStick.lowerButton(13).onTrue(new InstantCommand(arm::homeTelescope)); // Home telescope
        
        // Manually disable (kill) subsystems
        leftStick.lowerButton(7)
            .onTrue(new InstantCommand(arm::togglePivotActive));
        leftStick.lowerButton(6)
            .onTrue(new InstantCommand(arm::toggleTelescopeActive));
        leftStick.lowerButton(5)
            .onTrue(new InstantCommand(arm::toggleWristActive));
        leftStick.lowerButton(8)
            .onTrue(new InstantCommand(arm::toggleAllActive));
        
        // Set game piece mode
        masher.leftBumper().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.Cube)));           
        masher.rightBumper().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.ConeDown)));
        masher.y().onTrue(new InstantCommand(() ->
            RobotState.getInstance().setMode(GamePieceMode.ConeUp)));
        
        // Move arm
        masher.pov(0).onTrue(getMoveArmCommand(Position.High)); // move to high
        masher.pov(90).onTrue(getMoveArmCommand(Position.Mid)); // move to mid
        masher.pov(180).onTrue(getMoveArmCommand(Position.Ground)); // move to ground
        masher.pov(270).onTrue(getMoveArmCommand(Position.Stow)); // move to stow    
        masher.x().onTrue(getMoveArmCommand(Position.Shelf)); // move to shelf

        masher.a().onTrue(getWristDunkCommand(true)); // Dunk cone
        masher.b().onTrue(getWristDunkCommand(false)); // Un-dunk cone
       
        // Manually jog arm
        masher.leftStickLeft()
            .onTrue(new InstantCommand(arm::jogPivotBackward)); // jog pivot back
        masher.leftStickRight()
            .onTrue(new InstantCommand(arm::jogPivotForward)); // jog pivot forward

        masher.rightStickRight()
            .onTrue(new InstantCommand(arm::jogTelescopeIn)); // jog telescope in
        masher.rightStickLeft()
            .onTrue(new InstantCommand(arm::jogTelescopeOut)); // jog telescope out

        masher.rightStickUp()
            .onTrue(new InstantCommand(arm::jogWristBackward)); // jog wrist back
        masher.rightStickDown()
            .onTrue(new InstantCommand(arm::jogWristForward)); // jog wrist forward

        // Intake
        masher.leftTrigger()
            .onTrue(new InstantCommand(intake::toggleIntake)); // toggle intake

        masher.rightTrigger()
            .onTrue(new InstantCommand(intake::place)) // start place
            .onFalse(new InstantCommand(intake::stop)); // stop place

        masher.back()
            .onTrue(new InstantCommand(arm::homeWrist)); // home wrist

        masher.start()
            .onTrue(new InstantCommand(arm::invertActiveSide)); // flip side
    }

    private void configureAutos() {
        autoChooser.setDefaultOption("Blue clear 3", "B-1-1");
        autoChooser.addOption("Blue clear 2.5", "B-1-2");
        // autoChooser.addOption("B-1-3", "B-1-3");

        autoChooser.addOption("Blue middle balance", "B-2-1");

        // autoChooser.addOption("B-3-1", "B-3-1");
        // autoChooser.addOption("B-3-2", "B-3-2");

        autoChooser.addOption("Red clear 3", "R-1-1");
        autoChooser.addOption("Red clear 2.5", "R-1-2");
        // autoChooser.addOption("R-1-3", "R-1-3");

        autoChooser.addOption("Red middle balance", "R-2-1");

        // autoChooser.addOption("R-3-1", "R-3-1");
        // autoChooser.addOption("R-3-2", "R-3-2");

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return activeAutoCommand;
    }

    public void disabledPeriodic() {
        if (activeAutoCommand == null || !activeAutoName.equals(autoChooser.getSelected())) {
            activeAutoCommand = new AutoRoutines(autoChooser.getSelected(), drive, arm, intake);
            activeAutoName = autoChooser.getSelected();
        }

        drive.setBrakeMode(!brakeSwitch.get());
        arm.setBrakeMode(!brakeSwitch.get());

        ledManager.setOff(ledsSwitch.get());
    }

    public void enabledInit() {
        drive.setBrakeMode(true);
        arm.setBrakeMode(true);

        ledManager.setOff(false);

        if (DriverStation.isTeleop()) {
            arm.homeTelescope();
            if (!RobotState.getInstance().hasIntaked()) {
                arm.homeWrist();
            }
            // pivot.normalConstrain();
        }

        arm.setSetpoint(ArmPositions.stow);
    }

    private Command getWristDunkCommand(boolean up) {
        return arm.move(
            new ArmPosition(
                arm.getPosition().pivot(),
                arm.getPosition().telescope(),
                up ? ArmPositions.wristConePlace : ArmPositions.placeConeDownHigh.wrist()),
            false
        );
    }

    private Command getMoveArmCommand(Position position) {
        return new InstantCommand(() -> {
            ArmPosition setpoint = PositionHelper.getPosition(position, RobotState.getInstance().getMode());
            
            if (position == Position.High || position == Position.Mid) {
                ActiveArmSide side = Math.abs(drive.getRotation().getRadians()) < Math.PI / 2
                    ? ActiveArmSide.BACK : ActiveArmSide.FRONT;
                arm.setArmSide(side);
            }
            else if (position == Position.Shelf) {
                ActiveArmSide side = Math.abs(drive.getRotation().getRadians()) < Math.PI / 2
                    ? ActiveArmSide.BACK : ActiveArmSide.FRONT;
                arm.setArmSide(side);
            }
            
            // move
            arm.move(setpoint, false).schedule();
        });
    }

}
// </https://www.youtube.com/watch?v=mXlsbBgrKKY>
