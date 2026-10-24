package frc.robot

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitSeconds
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.constants.kCoDriverControllerPort
import frc.robot.constants.kDeadBandThreshold
import frc.robot.constants.kDriverControllerPort
import frc.robot.subsystems.Feeder
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.drivetrain.Drivetrain

@Logged
class RobotContainer : AutoCloseable {
    val driverController = CommandXboxController(kDriverControllerPort)
    val coDriverController = CommandXboxController(kCoDriverControllerPort)

    val drivetrain = Drivetrain()
    val intake = Intake()
    val indexer = Indexer()
    val feeder = Feeder()
    val shooter = Shooter()

    init {
        // config controller binds
        drivetrain.defaultCommand = drivetrain.drive(
            { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
            { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) },
            { -MathUtil.applyDeadband(driverController.rightX, kDeadBandThreshold) }
        )

        coDriverController.leftBumper()
            .whileTrue(indexer.indexer.alongWith(intake.intake).alongWith(feeder.reverse))
            .onFalse(
                indexer.indexerReverse.withTimeout(0.05)
                    .alongWith(shooter.shootReverse.withTimeout(0.1))
            )
            
        coDriverController.leftTrigger().whileTrue(indexer.indexerReverse)
        // coDriverController.a().whileTrue(intake.intake)

        coDriverController.rightBumper().whileTrue(shooter.shoot)
        
        coDriverController.rightTrigger().whileTrue(waitSeconds(0.5).andThen(indexer.indexer).alongWith(feeder.feed))

        // coDriverController.leftBumper().whileTrue(shooter.sysIdForwardQuasistatic)
        // coDriverController.rightBumper().whileTrue(shooter.sysIdReverseQuasistatic)

        // coDriverController.leftTrigger().whileTrue(shooter.sysIdForwardDynamic)
        // coDriverController.rightTrigger().whileTrue(shooter.sysIdReverseDynamic)

        driverController.leftBumper().whileTrue(
            drivetrain.drive(
                { 0.1 },
                { 0.0 },
                { 0.0 }
            ))

        driverController.rightBumper().whileTrue(
            drivetrain.drive(
                { 0.1 },
                { 0.0 },
                { 0.0 }
            ))
    }


    val autonomousCommand: Command
        get() {
            // return mPathChooser.selected36
            return Commands.none()
        }

    override fun close() {
        drivetrain.close()
        intake.close()
        indexer.close()
        feeder.close()
        shooter.close()
    }
}
