package frc.robot

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.alignToHeading
import frc.robot.commands.alignToPose
import frc.robot.commands.alignToTarget
import frc.robot.commands.intake
import frc.robot.commands.shoot
import frc.robot.commands.stage
import frc.robot.constants.kCoDriverControllerPort
import frc.robot.constants.kDeadBandThreshold
import frc.robot.constants.kDriverControllerPort
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.drivetrain.Vision

@Logged
class RobotContainer : AutoCloseable {
    val driverController = CommandXboxController(kDriverControllerPort)
    val coDriverController = CommandXboxController(kCoDriverControllerPort)

    val vision = Vision()

    val drivetrain = Drivetrain()
    val intake = Intake()
    val indexer = Indexer()
    val shooter = Shooter()

    init {
        // config controller binds
        drivetrain.defaultCommand = drivetrain.drive(
            { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
            { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) },
            { -MathUtil.applyDeadband(driverController.rightX, kDeadBandThreshold) }
        )

        driverController.leftBumper().whileTrue(
            drivetrain.alignToHeading(
                { Rotation2d.kZero },
                { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
                { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) },
            )
        )

//        driverController.leftBumper().whileTrue(
//            drivetrain.alignToPose(
//                { Pose2d.kZero }
//            )
//        )

        driverController.rightBumper().whileTrue(
            drivetrain.alignToTarget(
                { vision.bestTarget },
                { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
                { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) },
                { -MathUtil.applyDeadband(driverController.rightX, kDeadBandThreshold) }
            )
        )

        // micro adjustments
        driverController.povUp().whileTrue(drivetrain.drive({ 0.1 }, { 0.0 }, { 0.0 }))
        driverController.povRight().whileTrue(drivetrain.drive({ 0.0 }, { -0.1 }, { 0.0 }))
        driverController.povDown().whileTrue(drivetrain.drive({ -0.1 }, { 0.0 }, { 0.0 }))
        driverController.povLeft().whileTrue(drivetrain.drive({ 0.0 }, { 0.1 }, { 0.0 }))

        coDriverController.leftBumper()
            .whileTrue(intake(intake, indexer))
            .onFalse(stage(indexer, 0.15))

        coDriverController.leftTrigger().whileTrue(indexer.reverse)
        // coDriverController.a().whileTrue(intake.intake)

        coDriverController.rightBumper().whileTrue(shoot(indexer, intake, shooter))

        // coDriverController.leftBumper().whileTrue(shooter.sysIdForwardQuasistatic)
        // coDriverController.rightBumper().whileTrue(shooter.sysIdReverseQuasistatic)

        // coDriverController.leftTrigger().whileTrue(shooter.sysIdForwardDynamic)
        // coDriverController.rightTrigger().whileTrue(shooter.sysIdReverseDynamic)

        // driverController.leftBumper().whileTrue(
        //     drivetrain.drive(
        //         { 0.1 },
        //         { 0.0 },
        //         { 0.0 }
        //     ))

        // driverController.rightBumper().whileTrue(
        //     drivetrain.drive(
        //         { 0.1 },
        //         { 0.0 },
        //         { 0.0 }
        //     ))
    }


    val autonomousCommand: Command
        get() {
            return shoot(indexer, intake, shooter).withTimeout(6.0)
                .andThen(drivetrain.drive({ 0.5 }, { 0.0 }, { 0.0 }).withTimeout(1.0))
        }

    override fun close() {
        vision.close()

        drivetrain.close()
        intake.close()
        indexer.close()
        shooter.close()
    }
}
