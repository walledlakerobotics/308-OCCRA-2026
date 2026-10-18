package frc.robot.subsystems

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.kIntakeCANID
import frc.robot.constants.kIntakeerConfig

@Logged
class Intake : SubsystemBase(), AutoCloseable {
    val intakeMotor = TalonFX(kIntakeCANID)
    val request = VelocityVoltage(0.0)

    init {
        intakeMotor.configurator.apply(kIntakeerConfig)
    }

    fun motorIntake(velocity: AngularVelocity) {
        intakeMotor.setControl(request.withVelocity(velocity))
    }

    fun motorIntake(velocity: Double) {
        intakeMotor.setVoltage(velocity)
    }

    val intake: Command
        get() = startEnd({
            motorIntake(4.0)
        }, {
            motorIntake(0.0)
        })

    override fun periodic() {
        SmartDashboard.putNumber("Intake/Velocity", intakeMotor.velocity.value.`in`(RotationsPerSecond))

        // val kP = SmartDashboard.getNumber("Intake/kP", 0.0)
        // val kI = SmartDashboard.getNumber("Intake/kI", 0.0)
        // val kD = SmartDashboard.getNumber("Intake/kD", 0.0)

        // intakeMotor.configurator.apply(Slot0Configs().withKP())
    }

    override fun close() {
        intakeMotor.close()
    }
}
