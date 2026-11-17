package frc.robot

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.Autos
import frc.robot.commands.alignToBall
import frc.robot.commands.alignToHeading
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

    val drivetrain = Drivetrain()
    val intake = Intake()
    val indexer = Indexer()
    val shooter = Shooter()

    val vision = Vision()

    val autos = Autos(drivetrain, indexer, intake, shooter)

    init {
        SmartDashboard.putData("Autos/Select", autos.chooser)

        // config controller binds
        drivetrain.defaultCommand = drivetrain.drive(
            { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
            { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) },
            { -MathUtil.applyDeadband(driverController.rightX, kDeadBandThreshold) }
        )

        driverController.leftBumper().whileTrue(
            drivetrain.alignToHeading(
                { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
                { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) },
            ) { Rotation2d.kZero }
        )

        driverController.rightBumper().whileTrue(
            drivetrain.alignToBall(
                vision,
                { -MathUtil.applyDeadband(driverController.leftY, kDeadBandThreshold) },
                { -MathUtil.applyDeadband(driverController.leftX, kDeadBandThreshold) }
            )
        )

        // micro adjustments
        driverController.povUp().whileTrue(drivetrain.drive({ 0.1 }, { 0.0 }, { 0.0 }))
        driverController.povRight().whileTrue(drivetrain.drive({ 0.0 }, { -0.1 }, { 0.0 }))
        driverController.povDown().whileTrue(drivetrain.drive({ -0.1 }, { 0.0 }, { 0.0 }))
        driverController.povLeft().whileTrue(drivetrain.drive({ 0.0 }, { 0.1 }, { 0.0 }))

        coDriverController.leftBumper()
            .whileTrue(intake(intake, indexer, shooter))
            .onFalse(stage(indexer, 0.15))

        coDriverController.leftTrigger().whileTrue(indexer.reverse)

        coDriverController.rightBumper().whileTrue(shoot(indexer, intake, shooter))

        SmartDashboard.putNumber("Auto/Time", 1.2)
    }


    val autonomousCommand: Command
        get() {
//            return drivetrain.drive({ -0.5 }, { 0.0 }, { 0.0 })
//                .withTimeout(SmartDashboard.getNumber("Auto/Time", 1.2))
//                .andThen(shoot(indexer, intake, shooter).withTimeout(5.0))

            return autos.chooser.selected
        }

    override fun close() {
        vision.close()

        drivetrain.close()
        intake.close()
        indexer.close()
        shooter.close()
    }
}
