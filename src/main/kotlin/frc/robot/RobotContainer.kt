package frc.robot

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.constants.kCoDriverControllerPort
import frc.robot.constants.kDeadBandThreshold
import frc.robot.constants.kDriverControllerPort
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.drivetrain.MecanumDrive

@Logged
class RobotContainer {
    val driverController = CommandXboxController(kDriverControllerPort)
    val coDriverController = CommandXboxController(kCoDriverControllerPort)

    val drivetrain = MecanumDrive()
    val intake = Intake()
    val indexer = Indexer()
    val shooter = Shooter()
    
    init {
        // config controller binds. 
        drivetrain.defaultCommand = drivetrain.drive(
            { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
            { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) },
            { MathUtil.applyDeadband(driverController.rightX, kDeadBandThreshold) }
        )
        
        coDriverController.a().whileTrue(indexer.indexer.alongWith(intake.intake))
        
        coDriverController.b().whileTrue(shooter.shoot())
    }


    val autonomousCommand: Command
        get() {
            // return mPathChooser.selected36
            return Commands.none()
        }

}
