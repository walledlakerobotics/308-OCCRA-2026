package frc.robot.subsystems.drivetrain

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.constants.kEscapeByte
import frc.robot.constants.kEscapeByteMask
import frc.robot.constants.kFrameSignature
import kotlinx.io.*
import net.jpountz.xxhash.XXHashFactory
import java.util.concurrent.atomic.AtomicBoolean

@Logged
class Pinpoint(val port: SerialPort.Port) : AutoCloseable {
    // port serial instance
    private var serialPort =
        SerialPort(115200, port)

    private var serialPortSim = if (RobotBase.isSimulation()) {
        com.fazecast.jSerialComm.SerialPort.getCommPorts().firstOrNull {
            it.vendorID == 0x2E8A && it.productID == 0xF00F
        }
    } else null

    @Volatile
    private var odometryData = OdometryData()
    
    private var repeatedZeroBytes = 0
    
    var connected = false
        private set

    private val isRunning = AtomicBoolean(false)

    private val buffer = Buffer()
    private val dataSequence = sequence {
        while (true) {
            // Find the start of the frame
            @Suppress("ControlFlowWithEmptyBody")
            while (buffer.readUByteYielding() != kFrameSignature);

            val lengthTag = buffer.unescape(4).readUIntLe()
            val payload = buffer.unescape(lengthTag.toLong())
            val checksumTag = buffer.unescape(4).readUIntLe()

            val factory = XXHashFactory.safeInstance()

            val checksum = factory.hash32()
                .hash(payload.peek().readByteArray(), 0, payload.size.toInt(), 0)
                .toUInt()

            if (checksum != checksumTag) {
                continue
            }

            odometryData = OdometryData(payload)
        }
    }.iterator()

    // updates in seprate thread in different intervals
    private val notifier = Notifier {
        if (!isRunning.compareAndSet(false, true)) {
            return@Notifier
        }

        if (RobotBase.isReal()) {
            if (serialPort.bytesReceived == 0) {
                connected = false
                
                repeatedZeroBytes++
                    
                if (repeatedZeroBytes > 100) {
                    serialPort.close()

                    serialPort = SerialPort(115200, port)
                }
                
                isRunning.set(false)
                return@Notifier
            }

            connected = true
            repeatedZeroBytes = 0
            
            buffer.write(serialPort.read(serialPort.bytesReceived))
        } else {
            serialPortSim?.let { simPort ->
                try {
                    val bytes = ByteArray(simPort.bytesAvailable())

                    simPort.readBytes(bytes, bytes.size)

                    buffer.write(bytes)
                } catch (_: Exception) {
                    simPort.closePort()

                    serialPortSim = com.fazecast.jSerialComm.SerialPort.getCommPorts().firstOrNull {
                        it.vendorID == 0x2E8A && it.productID == 0xF00F
                    }
                    
                    serialPortSim?.openPort()

                    isRunning.set(false)
                    return@Notifier
                }
            }
        }

        try {
            dataSequence.next()
        } finally {
            isRunning.set(false)
        }
    }

    var position
        get() = odometryData.position
        set(value) = resetPosition(value)

    var heading
        get() = position.rotation!!
        set(value) = resetPosition(Pose2d(position.translation, value))
        
    val rotation
        get() = odometryData.rotation

    val velocity: ChassisSpeeds
        get() = odometryData.velocity

    init {
        serialPortSim?.apply {
            baudRate = 115200
            numDataBits = 8
            parity = com.fazecast.jSerialComm.SerialPort.NO_PARITY
            numStopBits = com.fazecast.jSerialComm.SerialPort.ONE_STOP_BIT

            openPort()
        }

        notifier.startPeriodic(0.01)
        
        SmartDashboard.putData("Odometry/ResetOdometry", Commands.runOnce({
            resetPosition(Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))
        }).ignoringDisable(true))
    }

    context(scope: SequenceScope<Unit>)
    private suspend fun Source.unescape(count: Long): Buffer {
        val escapedBuffer = Buffer()

        while (escapedBuffer.size < count) {
            var byte = readUByteYielding()
        
            if (byte == kEscapeByte) {
                val nextByte = readUByteYielding()
                
                byte = nextByte xor kEscapeByteMask
            }

            escapedBuffer.writeUByte(byte)
        }

        return escapedBuffer
    }

    context(scope: SequenceScope<Unit>)
    private suspend fun Source.readUByteYielding(): UByte {
        while (exhausted()) scope.yield(Unit)
        return readUByte()
    }
    
    private fun writeFrame(payload: Buffer) {
        val buffer = Buffer()
        
        val escapedPayload = payload.peek().escape()
        
        val lengthTag = Buffer().apply { writeUIntLe(payload.size.toUInt()) }.escape()
        
        val factory = XXHashFactory.safeInstance()
        val checksum = factory.hash32().hash(payload.peek().readByteArray(), 0, payload.size.toInt(), 0).toUInt()
        
        val checksumTag = Buffer().apply { writeUIntLe(checksum) }.escape()
        
        buffer.writeUByte(kFrameSignature)
        buffer.write(lengthTag, lengthTag.size)
        buffer.write(escapedPayload, escapedPayload.size)
        buffer.write(checksumTag, checksumTag.size)
        
        val bytes = buffer.readByteArray()
        
        if (RobotBase.isReal()) {
            serialPort.write(bytes, bytes.size)
        } else {
            serialPortSim?.writeBytes(bytes, bytes.size)
        }
    }
    
    private fun Source.escape(): Buffer {
        val escapedBuffer = Buffer()
        
        while (!exhausted()) {
            val byte = readUByte()
            
            if (byte == kEscapeByte || byte == kFrameSignature) {
                escapedBuffer.writeUByte(kEscapeByte)
                escapedBuffer.writeUByte(byte xor kEscapeByteMask)
                
                continue
            }
            
            escapedBuffer.writeUByte(byte)
        }
        
        return escapedBuffer
    }
    
    fun recalibrateIMU() {
        val data = ResetData(
            calibrateIMU = true,
            setPosition = false,
            positionX = 0.0,
            positionY = 0.0,
            positionHeading = 0.0
        )
        writeFrame(data.toBuffer())
    }
    
    fun resetPosition(position: Pose2d) {
        val data = ResetData(
            calibrateIMU = false,
            setPosition = true,
            positionX = position.measureX.`in`(Millimeters),
            positionY = position.measureY.`in`(Millimeters),
            positionHeading = position.rotation.degrees
        )
        writeFrame(data.toBuffer())
    }
    
    override fun close() {
        serialPort.close()
        serialPortSim?.closePort()
        
        notifier.close()
    }
}

@Logged
private data class OdometryData(
    val encoderX: UInt,
    val encoderY: UInt,
    
    val positionX: Double,
    val positionY: Double,
    
    val quaternionW: Double,
    val quaternionX: Double,
    val quaternionY: Double,
    val quaternionZ: Double,
    
    val velocityX: Double,
    val velocityY: Double,
    val velocityHeading: Double
) {
    val position: Pose2d
        get() = Pose2d(
            Translation2d(Millimeters.of(positionX), Millimeters.of(positionY)),
            heading
        )
        
    val heading: Rotation2d
        get() = rotation.toRotation2d()
        
    val rotation: Rotation3d
        get() = Rotation3d(quaternion)
        
    val quaternion: Quaternion
        get() = Quaternion(quaternionW, quaternionX, quaternionY, quaternionZ)

    val velocity: ChassisSpeeds
        get() = ChassisSpeeds(
            Millimeters.per(Second).of(velocityX),
            Millimeters.per(Second).of(velocityY),
            DegreesPerSecond.of(velocityHeading)
        )

    val velocityFieldRelative: ChassisSpeeds
        get() = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, position.rotation)

    constructor(
        encoderX: UInt,
        encoderY: UInt,
        positionX: Float,
        positionY: Float,
        quaternionW: Float,
        quaternionX: Float,
        quaternionY: Float,
        quaternionZ: Float,
        velocityX: Float,
        velocityY: Float,
        velocityHeading: Float
    ) : this(
        encoderX,
        encoderY,
        positionX.toDouble(),
        positionY.toDouble(),
        quaternionW.toDouble(),
        quaternionX.toDouble(),
        quaternionY.toDouble(),
        quaternionZ.toDouble(),
        velocityX.toDouble(),
        velocityY.toDouble(),
        velocityHeading.toDouble()
    )
    
    constructor(buffer: Buffer) : this(
        buffer.readUIntLe(),
        buffer.readUIntLe(),    
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe(),
        buffer.readFloatLe()
    )

    constructor() : this(0u, 0u, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
}

private data class ResetData(
    val calibrateIMU: Boolean,
    val setPosition: Boolean,
    val positionX: Float,
    val positionY: Float,
    val positionHeading: Float
) {
    fun toBuffer(): Buffer {
        val buffer = Buffer()

        buffer.writeUByte(if (calibrateIMU) 1u else 0u)
        buffer.writeUByte(if (setPosition) 1u else 0u)
        buffer.writeFloatLe(positionX)
        buffer.writeFloatLe(positionY)
        buffer.writeFloatLe(positionHeading)

        return buffer
    }
    
    constructor(
        calibrateIMU: Boolean,
        setPosition: Boolean,
        positionX: Double,
        positionY: Double,
        positionHeading: Double
    ) : this(
        calibrateIMU,
        setPosition,
        positionX.toFloat(),
        positionY.toFloat(),
        positionHeading.toFloat()
    )
    
    constructor() : this(false, false, 0.0f, 0.0f, 0.0f)
}
