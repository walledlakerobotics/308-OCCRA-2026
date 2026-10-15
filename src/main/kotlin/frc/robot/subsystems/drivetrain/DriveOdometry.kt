package frc.robot.subsystems.drivetrain

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.SerialPort
import frc.robot.constants.kEscapeByte
import frc.robot.constants.kFrameSignature
import kotlinx.io.Buffer
import kotlinx.io.Source
import kotlinx.io.readFloatLe
import kotlinx.io.readUByte
import kotlinx.io.readUIntLe
import kotlinx.io.writeUByte
import java.util.concurrent.atomic.AtomicBoolean

@Logged
class DriveOdometry(val port: SerialPort.Port) : AutoCloseable {


    // TODO: Need to make a sum check

    // port serial instance
    private var serialPort =
        SerialPort(115200, port)

    @Volatile
    private var odometryData = OdometryData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    private val isRunning = AtomicBoolean(false)


    private val buffer = Buffer()
    private val dataSequence = sequence {
        while (true) {
            // Find the start of the frame
            @Suppress("ControlFlowWithEmptyBody")
            while (buffer.readUByte() != kFrameSignature) {}

            val lengthTag = buffer.escape(4).readUIntLe()
            val payload = buffer.escape(lengthTag.toLong())

            odometryData = OdometryData(
                payload.readFloatLe(),
                payload.readFloatLe(),
                payload.readFloatLe(),
                payload.readFloatLe(),
                payload.readFloatLe(),
                payload.readFloatLe()
            )
        }
    }.iterator()

    // updates in seprate thread in different intervals
    private val notifier = Notifier {
        if (!isRunning.compareAndSet(false, true)) {
            return@Notifier
        }
        
        try {
            buffer.write(serialPort.read(serialPort.bytesReceived))
        } catch (_: Exception) {
            serialPort.close()

            serialPort = SerialPort(115200, port)

            isRunning.set(false)
            return@Notifier
        }
            
        try {
            dataSequence.next()
        } finally {
            isRunning.set(false)
        }
    }
    
    context(scope: SequenceScope<Unit>)
    private suspend fun Source.escape(count: Long): Buffer {
        val escapedBuffer = Buffer()
        
        while (escapedBuffer.size < count) {
            readUByte().let { byte ->
                if (byte == kEscapeByte) {
                    val nextByte = readUByte()
                    escapedBuffer.writeUByte(nextByte xor 0x20u)
                    
                    continue
                }
                
                escapedBuffer.writeUByte(byte)
            }
        }
        
        return escapedBuffer
    }
    
    context(scope: SequenceScope<Unit>)
    private suspend fun Source.readUByte(): UByte {
        while (exhausted()) scope.yield(Unit)
        return readUByte()
    }

    val position: Pose2d
        get() = odometryData.position

    val velocity: ChassisSpeeds
        get() = odometryData.velocity

    init {
        notifier.startPeriodic(0.01)
    }

    override fun close() {
        serialPort.close()
        notifier.close()
    }
}

data class OdometryData(
    val positionX: Double,
    val positionY: Double,
    val positionHeading: Double,
    val velocityX: Double,
    val velocityY: Double,
    val velocityHeading: Double
) {
    val position: Pose2d
        get() = Pose2d(
            Translation2d(positionX, positionY),
            Rotation2d.fromRadians(positionHeading)
        )
        
    val velocity: ChassisSpeeds
        get() = ChassisSpeeds(
            velocityX,
            velocityY,
            velocityHeading
        )
        
    val velocityFieldRelative: ChassisSpeeds
        get() = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, position.rotation)
        
    constructor(
        positionX: Float,
        positionY: Float,
        positionHeading: Float,
        velocityX: Float,
        velocityY: Float,
        velocityHeading: Float
    ) : this(
        positionX.toDouble(),
        positionY.toDouble(),
        positionHeading.toDouble(),
        velocityX.toDouble(),
        velocityY.toDouble(),
        velocityHeading.toDouble()
    )
        
    constructor() : this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
}


