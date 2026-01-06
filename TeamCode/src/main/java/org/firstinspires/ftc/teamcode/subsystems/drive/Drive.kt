package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.lateralFactor
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import kotlin.math.absoluteValue

@Config
class Drive(hwMap: HardwareMap) {
    val localizer = Localizer(hwMap)

    companion object DriveConstants {
        @JvmField var lateralFactor = 0.7

        @JvmField var kS = 0.145
        @JvmField var kV = 0.0151

        @JvmField var kA = 0.005
        @JvmField var hP = 2.0
        @JvmField var hD = 0.15
        @JvmField var hT = 0.02
        @JvmField var xyP = 0.13
        @JvmField var xyD = 0.04
        @JvmField var xyT = 0.5
        @JvmField var tipAccelForward = 230.0
        @JvmField var tipAccelBackward = -150.0
    }
    private val flMotor: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "fl"), true, 0.01)
    fun setFlPower(power: Double) { flMotor.power = power }

    private val frMotor: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "fr"), true, 0.01)
    fun setFrPower(power: Double) { frMotor.power = power }

    private val blMotor: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "bl"), true, 0.01)
    fun setBlPower(power: Double) { blMotor.power = power }

    private val brMotor: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "br"), true, 0.01)
    fun setBrPower(power: Double) { brMotor.power = power }

    private fun setZeroPowerBehaviours(zeroPowerBehavior: ZeroPowerBehavior) {
        flMotor.zeroPowerBehavior = zeroPowerBehavior
        frMotor.zeroPowerBehavior = zeroPowerBehavior
        blMotor.zeroPowerBehavior = zeroPowerBehavior
        brMotor.zeroPowerBehavior = zeroPowerBehavior
    }

    init {
        flMotor.direction = Direction.REVERSE
        frMotor.direction = Direction.FORWARD
        blMotor.direction = Direction.REVERSE
        brMotor.direction = Direction.FORWARD
        setZeroPowerBehaviours(ZeroPowerBehavior.FLOAT)
    }
    // Drive math, etc

    var follow: () -> DriveVectors = getPointToPoint(Pose(0.0, 0.0, 0.0), localizer)

    fun startP2PWithTargetPose(targetPose: Pose) {
        follow = getPointToPoint(targetPose, localizer)
    }

    fun update() {
        val driveVectors = follow()
        val leftPowers = driveVectors.getLeftPowers()
        val rightPowers = driveVectors.getRightPowers()

        flMotor.power = leftPowers.first
        frMotor.power = rightPowers.first
        blMotor.power = leftPowers.second
        brMotor.power = rightPowers.second
    }

    fun goToCircle(
        pose: Pose,
        distanceTolerance: Double = xyT,
        headingTolerance: Double = hT,
    ) = Sequence(
        Instant {startP2PWithTargetPose(pose)},
        WaitUntil {
            (localizer.pose - pose).inCircle(distanceTolerance, headingTolerance) &&
            localizer.poseVel.inCircle(0.5, 0.04)
        },
    )
    fun doWheelie(intake: Intake) = Sequence(
        Instant {
            localizer.setWheelieBulkreadScope()
            follow = { DriveVectors.fromTranslation(Vector.fromCartesian(5.0, 0.0)) }
        },
        WaitUntil {
            getRelativeVelocity(localizer.pose, localizer.poseVel).x >= 60.0
        },
        Instant {
            intake.behaviour = Intake.IntakeBehaviour.Wheelie(flMotor, frMotor)
            follow = { DriveVectors.fromTranslation(Vector.fromCartesian(-2.0, 0.0)) }
        },
        WaitUntil {
            localizer.pinpoint.getPitch(AngleUnit.RADIANS) > 0.7
        },
        Instant {
            val startTime = System.currentTimeMillis()
            follow = {
                val t: Double = (System.currentTimeMillis() - startTime)/800.0
                DriveVectors.fromTranslation(Vector.fromCartesian(-1.0 * (1 - t), 0.0))
            }
        },
        Sleep(0.8),
        Instant {
            follow = { DriveVectors.fromTranslation(Vector.fromCartesian(0.0, 0.0))}
        },
        Sleep(0.5),
        Instant {
            localizer.setDefaultBulkreadScope()
        }
    )
    fun goToSquare(
        pose: Pose,
        xTolerance: Double = xyT,
        yTolerance: Double = xyT,
        headingTolerance: Double = hT,
    ) = Sequence(
        Instant {startP2PWithTargetPose(pose)},
        WaitUntil {(localizer.pose - pose).inSquare(xTolerance, yTolerance, headingTolerance) && localizer.poseVel.inCircle(0.5, 0.04) },
    )

    fun inShootableZone(): Boolean{
        return true
    }
}

data class DriveVectors(val left: Vector, val right: Vector) {
    fun trimmed(maxPower: Double): DriveVectors {
        val leftPowers = getLeftPowers()
        val rightPowers = getRightPowers()
        val scaleFactor = maxPower / maxOf(
            leftPowers.first.absoluteValue,
            leftPowers.second.absoluteValue,
            rightPowers.first.absoluteValue,
            rightPowers.second.absoluteValue,
            maxPower
        )
        return DriveVectors(left * scaleFactor, right * scaleFactor)
    }

    fun addWithoutPriority(other: DriveVectors, maxPower: Double = 1.0) = (this + other).trimmed(maxPower)

    fun addWithPriority(addedVectors: DriveVectors): DriveVectors {
        TODO()
    }

    operator fun plus(other: DriveVectors) = DriveVectors(this.left + other.left, this.right + other.right)

    fun getLeftPowers() = getSidePowers(left, getWheelVector(true, true), getWheelVector(false, true))
    fun getRightPowers() = getSidePowers(right, getWheelVector(true, false), getWheelVector(false, false))
    private fun getSidePowers(targetVector: Vector, front: Vector, back: Vector) = Pair(
            // Math derived from the following
            // Ax = b
            // x = (A^-1)b
            // A is the matrix describing the wheel vectors, b is the target vector , x is the output powers
            (front.x * targetVector.y - targetVector.x * front.y) / (front.x * back.y - back.x * front.y),
            (back.x * targetVector.y - targetVector.x * back.y) / (back.x * front.y - front.x * back.y)
        )

    companion object {
        fun fromTranslation(v: Vector) = DriveVectors(v, v)
        fun fromRotation(v: Double) = DriveVectors(Vector.fromCartesian(-v, 0.0), Vector.fromCartesian(v, 0.0))

        fun getWheelVector(front: Boolean, left: Boolean) = Vector.fromCartesian(
            1.0,
            if ((front && left) || (!front && !left)) lateralFactor else -lateralFactor
        ).norm()

        fun getTranslationalVectors(drive: Double, strafe: Double) = DriveVectors(
            left = Vector.fromCartesian(drive, strafe),
            right = Vector.fromCartesian(drive, strafe)
        )

        fun getHeadingVectors(turn: Double) = DriveVectors(
            left = Vector.fromCartesian(-turn, 0.0),
            right = Vector.fromCartesian(turn, 0.0)
        )
    }

    override fun toString() = "[L $left, R $right]"
}