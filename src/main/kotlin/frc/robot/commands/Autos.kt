package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.BLine.Path
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utilities.Auto
import frc.robot.utilities.Default
import frc.robot.utilities.createAutoChooser

class Autos(
    private val drivetrain: Drivetrain,
    private val indexer: Indexer,
    private val intake: Intake,
    private val shooter: Shooter
) {
    @Auto("Test")
    @Default
    val test
        get() = drivetrain.followPath(Path("Grab Ball"), true)

    @Auto("Shoot 1")
    val shoot1
        get() = drivetrain.followPath(Path("Shoot 1"), true)
            .andThen(shoot(indexer, intake, shooter).withTimeout(3.0))

    @Auto("Shoot 2")
    val shoot2
        get() = shoot1
            .andThen(drivetrain.followPath(Path("Grab Ball"))
                .deadlineFor(intake(intake, indexer, shooter)))
            .andThen(drivetrain.followPath(Path("Shoot 2")))
            .andThen(shoot(indexer, intake, shooter).withTimeout(3.0))

    val chooser by lazy { createAutoChooser(this) }
}