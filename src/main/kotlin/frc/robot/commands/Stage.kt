package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitSeconds
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

fun intake(intake: Intake, indexer: Indexer) = Commands.parallel(
    intake.run,
    indexer.run
)!!

fun stage(indexer: Indexer, timeout: Double) = indexer.reverse.withTimeout(timeout)!!

fun Indexer.runWhile(condition: () -> Boolean) =
    run.onlyWhile(condition)
        .andThen(idle().withTimeout(0.5))
        .repeatedly()!!

fun shoot(indexer: Indexer, intake: Intake, shooter: Shooter) = Commands.parallel(
    indexer.runWhile { shooter.atGoal },
    intake.run,
    shooter.shoot
)!!