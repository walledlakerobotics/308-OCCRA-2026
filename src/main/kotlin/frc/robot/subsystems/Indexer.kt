package frc.robot.subsystems

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.*

@Logged
class Indexer : SubsystemBase(), AutoCloseable {
    private val leftMotor = SparkMax(kLeftIndexerCANID, SparkLowLevel.MotorType.kBrushed)
    private val rightMotor = SparkMax(kRightIndexerCANID, SparkLowLevel.MotorType.kBrushed)

    private val leftSensor = DigitalInput(kLeftSensorChannel)
    private val rightSensor = DigitalInput(kRightSensorChannel)

    val leftSensorTriggered: Boolean
        get() = !leftSensor.get()

    val rightSensorTriggered: Boolean
        get() = !rightSensor.get()

    init {
        leftMotor.configure(kLeftIndexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightMotor.configure(kRightIndexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    val indexer: Command
        get() = startEnd({
            leftMotor.set(0.5)
            rightMotor.set(0.5)
        }, {
            leftMotor.set(0.0)
            rightMotor.set(0.0)
        })

    val indexerFast: Command
        get() = startEnd({
            leftMotor.set(1.0)
            rightMotor.set(1.0)
        }, {
            leftMotor.set(0.0)
            rightMotor.set(0.0)
        })

    val indexerReverse: Command
        get() = startEnd({
            leftMotor.set(-0.5)
            rightMotor.set(-0.5)
        }, {
            leftMotor.set(0.0)
            rightMotor.set(0.0)
        })

    override fun close() {
        leftMotor.close()
        rightMotor.close()

        leftSensor.close()
        rightSensor.close()
    }
}