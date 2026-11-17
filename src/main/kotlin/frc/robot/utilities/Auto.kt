package frc.robot.utilities

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import kotlin.reflect.KClass
import kotlin.reflect.KVisibility
import kotlin.reflect.full.createType
import kotlin.reflect.full.findAnnotation
import kotlin.reflect.full.functions
import kotlin.reflect.full.hasAnnotation
import kotlin.reflect.full.isSubtypeOf
import kotlin.reflect.full.memberFunctions
import kotlin.reflect.full.memberProperties
import kotlin.reflect.full.staticFunctions
import kotlin.reflect.full.staticProperties

private val COMMAND_TYPE = Command::class.createType()

@Target(AnnotationTarget.PROPERTY, AnnotationTarget.FUNCTION)
@Retention(AnnotationRetention.RUNTIME)
annotation class Auto(val name: String)

@Target(AnnotationTarget.PROPERTY, AnnotationTarget.FUNCTION)
@Retention(AnnotationRetention.RUNTIME)
annotation class Default

fun createAutoChooser(obj: Any): SendableChooser<Command> {
    val chooser = SendableChooser<Command>()

    (obj::class.memberFunctions + obj::class.memberProperties).filter {
        it.returnType.isSubtypeOf(COMMAND_TYPE) &&
        it.parameters.size == 1 &&
        it.visibility == KVisibility.PUBLIC &&
        it.hasAnnotation<Auto>()
    }.forEach {
        val name = it.findAnnotation<Auto>()!!.name
        val command = it.call(obj) as Command

        if (it.hasAnnotation<Default>()) {
            chooser.setDefaultOption(name, command)

            return@forEach
        }

        chooser.addOption(name, command)
    }

    return chooser
}