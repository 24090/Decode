package org.firstinspires.ftc.teamcode.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.clamp
import org.firstinspires.ftc.teamcode.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.controlsystems.SquID
import kotlin.math.min

@Config
class Drive(hwMap: HardwareMap) {
    val localizer = Localizer(hwMap)

    companion object DriveConstants{
        @JvmField var lateralFactor = 0.8
        @JvmField var kSqH = 0.8;
        @JvmField var kPT = 0.2;
        @JvmField var kDT = 0.03;
        @JvmField var kLT = 0.3;
        @JvmField var threshT = 0.3;
    }

    var targetPose = Pose(0.0, 0.0, 0.0)
    val error
        get() = localizer.fieldPoseToRelative(targetPose)
    val dError: Pose
        get(){
            val translation = Vector.fromPose(localizer.poseVel).rotated(-localizer.heading)
            return -Pose(translation.x, translation.y, localizer.headingVel)
        }
    private val flMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java, "flMotor"));
        fun setFlPower(power: Double){flMotor.power = power}
    private val frMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java, "frMotor"));
        fun setFrPower(power: Double){frMotor.power = power}
    private val blMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java,"blMotor"));
        fun setBlPower(power: Double){blMotor.power = power}
    private val brMotor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java,"brMotor"));
        fun setBrPower(power: Double){brMotor.power = power}

    private fun setZeroPowerBehaviours(zeroPowerBehavior: ZeroPowerBehavior){
        flMotor.zeroPowerBehavior = zeroPowerBehavior
        frMotor.zeroPowerBehavior = zeroPowerBehavior
        blMotor.zeroPowerBehavior = zeroPowerBehavior
        brMotor.zeroPowerBehavior = zeroPowerBehavior
    }

    var drive = 0.0;
    var strafe = 0.0;
    var turn = 0.0;

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
    fun atTargetSquare(xTolerance: Double, yTolerance: Double, headingTolerance: Double): Boolean{
        return localizer.pose.inSquare(targetPose, xTolerance, yTolerance, headingTolerance)
    }

    // Drive math, etc

    fun setMotorPowers(){
        val driveVectors = getHeadingVectors().addNormalized(getTranslationalVectors())
        val leftPowers = getSidePowers(driveVectors.left, getWheelVector(true, true), getWheelVector(false, true))
        val rightPowers = getSidePowers(driveVectors.right, getWheelVector(true, false), getWheelVector(false, false))

        flMotor.power = leftPowers.first
        frMotor.power = rightPowers.first
        blMotor.power = leftPowers.second
        brMotor.power = rightPowers.second
    }

    fun update(){
        localizer.update()

        turn = SquID(AngleUnit.normalizeRadians(error.heading), kSqH)
        val translational = PDLT(Vector.fromPose(error), Vector.fromPose(dError), kPT, kDT, kLT, threshT)
        drive = if (translational.x.isNaN()) 0.0 else translational.x
        strafe = if (translational.y.isNaN()) 0.0 else translational.y
        setMotorPowers()
    }

    // drive vector calculations

    private data class DriveVectors(val left: Vector, val right: Vector){
        fun addNormalized(addedVectors: DriveVectors): DriveVectors {
            // extra space available for left vector
            val extraLeft = 1 - this.left.length
            // extra space available for right vector
            val extraRight = 1 - this.right.length
            // the scaling that the left vector needs to have length <= extraLeft
            val scaleLeft =
                if (addedVectors.left.length != 0.0)
                    clamp(addedVectors.left.length, 0.0, extraLeft)/addedVectors.left.length
                else
                    1.0
            // the scaling that the right vector needs to have length <= extraRight
            val scaleRight =
                if (addedVectors.right.length != 0.0)
                    clamp(addedVectors.right.length, 0.0, extraRight)/addedVectors.right.length
                else
                    1.0
            val scale = min(scaleLeft, scaleRight)
            return DriveVectors(
                this.left + addedVectors.left * scale,
                this.right + addedVectors.right * scale
            )
        }
        override fun toString(): String {
            return "[L $left, R $right]"
        }
    }

    private fun getTranslationalVectors() = DriveVectors(
        left = Vector.fromCartesian(drive, strafe).normalized(),
        right = Vector.fromCartesian(drive, strafe).normalized()
    )

    private fun getHeadingVectors() = DriveVectors(
        left = Vector.fromCartesian(-turn, 0.0).normalized(),
        right = Vector.fromCartesian(turn, 0.0).normalized()
    )

    private fun getWheelVector(front: Boolean, left: Boolean) = Vector.fromCartesian(
        1.0,
        if ((front && left) || (!front && !left)) lateralFactor else -lateralFactor
    ).norm()

    private fun getSidePowers(targetVector: Vector, wheelVectorA: Vector, wheelVectorB: Vector) = Pair(
        // Math derived from the following
        // Ax = b
        // x = (A^-1)b
        // A is the matrix describing the wheel vectors, b is the target vector , x is the output powers
        (wheelVectorA.x*targetVector.y - targetVector.x*wheelVectorA.y) / (wheelVectorA.x*wheelVectorB.y - wheelVectorB.x*wheelVectorA.y),
        (wheelVectorB.x*targetVector.y - targetVector.x*wheelVectorB.y) / (wheelVectorB.x*wheelVectorA.y - wheelVectorA.x*wheelVectorB.y)
    )
}