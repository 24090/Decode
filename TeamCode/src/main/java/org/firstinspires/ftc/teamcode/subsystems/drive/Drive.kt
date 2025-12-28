package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.lateralFactor
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.controlHubVoltage

@Config
class Drive(hwMap: HardwareMap) {
    val localizer = Localizer(hwMap)

    companion object DriveConstants {
        @JvmField var lateralFactor = 0.7
        @JvmField var hP = 2.0
        @JvmField var hD = 0.15
        @JvmField var hL = 0.19
        @JvmField var hT = 0.02
        @JvmField var xyP = 0.13
        @JvmField var xyD = 0.04
        @JvmField var xyL = 0.19
        @JvmField var xyT = 0.5
    }
    
    var targetPose = Pose(0.0, 0.0, 0.0)
    val error
        get() = localizer.fieldPoseToRelative(targetPose)
    val dError: Pose
        get() {
            val translation = Vector.fromPose(localizer.poseVel).rotated(-localizer.heading)
            return -Pose(translation.x, translation.y, localizer.headingVel)
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

    var drive = 0.0
    var strafe = 0.0
    var turn = 0.0

    init {
        flMotor.direction = Direction.REVERSE
        frMotor.direction = Direction.FORWARD
        blMotor.direction = Direction.REVERSE
        brMotor.direction = Direction.FORWARD
        setZeroPowerBehaviours(ZeroPowerBehavior.FLOAT)
    }

    // Util

    fun atTargetCircle(distanceTolerance: Double, headingTolerance: Double): Boolean {
        return localizer.pose.inCircle(targetPose, distanceTolerance, headingTolerance)
    }

    fun atTargetSquare(xTolerance: Double, yTolerance: Double, headingTolerance: Double): Boolean {
        return localizer.pose.inSquare(targetPose, xTolerance, yTolerance, headingTolerance)
    }

    // Drive math, etc

    private fun setMotorPowers() {
        val driveVectors = getHeadingVectors().addWithoutPriority(getTranslationalVectors(), controlHubVoltage/14.0)
        val leftPowers = driveVectors.getLeftPowers()
        val rightPowers = driveVectors.getRightPowers()

        flMotor.power = leftPowers.first
        frMotor.power = rightPowers.first
        blMotor.power = leftPowers.second
        brMotor.power = rightPowers.second
    }

    fun updateHeading(){
        turn = PDLT(AngleUnit.normalizeRadians(error.heading), dError.heading, hP, hD, hL, hT)
    }
    var currentUpdateHeading: () -> Unit = ::updateHeading
    fun updateTranslational(){
        val translational =
            PDLT(Vector.fromPose(error), Vector.fromPose(dError), xyP, xyD, xyL, xyT)
        drive = if (translational.x.isNaN()) 0.0 else translational.x
        strafe = if (translational.y.isNaN()) 0.0 else translational.y
    }
    var currentUpdateTranslational: () -> Unit = ::updateTranslational

    fun update() {
        currentUpdateHeading()
        currentUpdateTranslational()
        setMotorPowers()
        flMotor.direction
    }


    // drive vector calculations

    private fun getTranslationalVectors() = DriveVectors(
        left = Vector.fromCartesian(drive, strafe),
        right = Vector.fromCartesian(drive, strafe)
    )

    private fun getHeadingVectors() = DriveVectors(
        left = Vector.fromCartesian(-turn, 0.0),
        right = Vector.fromCartesian(turn, 0.0)
    )

    fun goToCircle(
        pose: Pose,
        distanceTolerance: Double = xyT,
        headingTolerance: Double = 0.04,
    ) = Sequence(
        Instant {
            targetPose = pose
        },
        WaitUntil { atTargetCircle(distanceTolerance, headingTolerance) && localizer.poseVel.inCircle(0.5, 0.04)},
    )
    fun goToSquare(
        pose: Pose,
        xTolerance: Double = xyT,
        yTolerance: Double = xyT,
        headingTolerance: Double = 0.04,
    ) = Sequence(
        Instant {
            targetPose = pose
        },
        WaitUntil { atTargetSquare(xTolerance, yTolerance, headingTolerance) },
    )

    fun inShootableZone(): Boolean{
        return true
    }
}

data class DriveVectors(val left: Vector, val right: Vector) {
    fun trimmed(maxPower: Double): DriveVectors {
        val leftPowers = getLeftPowers()
        val rightPowers = getRightPowers()
        val scaleFactor = maxPower/maxOf(leftPowers.first, leftPowers.second, rightPowers.first, rightPowers.second, maxPower)
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
    }

    override fun toString() = "[L $left, R $right]"
}