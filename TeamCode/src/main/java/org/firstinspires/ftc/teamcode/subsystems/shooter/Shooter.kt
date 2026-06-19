package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.InterpolatedLUT
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.ShootCounter
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import kotlin.math.abs
import kotlin.math.sqrt

val distanceToVelocityLeftLUT = InterpolatedLUT(mapOf(
    Pair(0.0, 1030.0), // 0 in
    Pair(36*sqrt(2.0), 1490.0 - 25.0 ), // 36 sqrt 2 in
    Pair(48*sqrt(2.0), 1470.0 - 25.0), // 48 sqrt 2 in
    Pair(60*sqrt(2.0), 1450.0 - 25.0),  // 60 sqrt 2 in
    Pair(72*sqrt(2.0), 1450.0 - 25.0),  // 72 sqrt 2 in
    Pair(84*sqrt(2.0), 1590.0 - 25.0),  // 84 sqrt 2 in
    Pair(96*sqrt(2.0), 1750.0 - 25.0),  // 96 sqrt 2 in
    Pair(108*sqrt(2.0), 1700.0 - 25.0),
    Pair(120*sqrt(2.0), 1800.0 - 25.0),
))
val distanceToVelocityRightLUT = distanceToVelocityLeftLUT

val secondaryDistanceToVelocityLeftLUT = InterpolatedLUT(mapOf(
    Pair(0.0, 1040.0), // 0 in
    Pair(36*sqrt(2.0), 1290.0 - 15.0 ), // 36 sqrt 2 in
    Pair(48*sqrt(2.0), 1280.0 - 15.0), // 48 sqrt 2 in
    Pair(60*sqrt(2.0), 1260.0 - 15.0),  // 60 sqrt 2 in
    Pair(72*sqrt(2.0), 1320.0 - 15.0),  // 72 sqrt 2 in
    Pair(84*sqrt(2.0), 1500.0 - 15.0),  // 84 sqrt 2 in
    Pair(96*sqrt(2.0), 1550.0 - 15.0),  // 96 sqrt 2 in
    Pair(108*sqrt(2.0), 1600.0 - 15.0),
    Pair(120*sqrt(2.0), 1700.0 - 15.0),
))
val secondaryDistanceToVelocityRightLUT = distanceToVelocityLeftLUT

val distanceToAngleLUT = InterpolatedLUT(mapOf(
    Pair(0.0, 1.0), // 0 in
    Pair(36*sqrt(2.0), 0.15), // 36 sqrt 2 in
    Pair(48*sqrt(2.0), 0.3), // 48 sqrt 2 in
    Pair(60*sqrt(2.0), 0.3),  // 60 sqrt 2 in
    Pair(72*sqrt(2.0), 0.3),  // 72 sqrt 2 in
    Pair(84*sqrt(2.0), 0.5),  // 84 sqrt 2 in
    Pair(96*sqrt(2.0), 0.6),  // 96 sqrt 2 in
    Pair(108*sqrt(2.0), 0.5),
    Pair(120*sqrt(2.0), 0.5),
))
val secondaryDistanceToAngleLUT = InterpolatedLUT(mapOf(
    Pair(0.0, 1.0), // 0 in
    Pair(36*sqrt(2.0), 0.0), // 36 sqrt 2 in
    Pair(48*sqrt(2.0), 0.0), // 48 sqrt 2 in
    Pair(60*sqrt(2.0), 0.0),  // 60 sqrt 2 in
    Pair(72*sqrt(2.0), 0.0),  // 72 sqrt 2 in
    Pair(84*sqrt(2.0), 0.1),  // 84 sqrt 2 in
    Pair(96*sqrt(2.0), 0.2),  // 96 sqrt 2 in
    Pair(108*sqrt(2.0), 0.222),
    Pair(120*sqrt(2.0), 0.2),
))

val exitVelocityToLeftVelocityLUT = InterpolatedLUT(mapOf(
    Pair(0.0, 0.0), // 0 in
    Pair(181.1681243438419, 1355.0), // 36 sqrt 2 in
    Pair(198.54371227569945, 1300.0), // 48 sqrt 2 in
    Pair(215.6597144491923, 1390.0),  // 60 sqrt 2 in
    Pair(231.97155294255114, 1520.0),  // 72 sqrt 2 in
    Pair(247.3880987428304, 1620.0),  // 84 sqrt 2 in
    Pair(262.0039126511864, 1730.0),  // 96 sqrt 2 in
    Pair(120 * sqrt(2.0), 2040.0),  // 108 sqrt 2 in
))

val exitVelocityToRightVelocityLUT = InterpolatedLUT(mapOf(
    Pair(0.0, 0.0), // 0 in
    Pair(181.1681243438419, 1355.0), // 36 sqrt 2 in
    Pair(198.54371227569945, 1300.0), // 48 sqrt 2 in
    Pair(215.6597144491923, 1390.0),  // 60 sqrt 2 in
    Pair(231.97155294255114, 1520.0),  // 72 sqrt 2 in
    Pair(247.3880987428304, 1620.0),  // 84 sqrt 2 in
    Pair(262.0039126511864, 1730.0),  // 96 sqrt 2 in
    Pair(275.90206925170855, 1960.0),  // 108 sqrt 2 in
    Pair(120 * sqrt(2.0), 2040.0),  // 120 sqrt 2 in

))

@Config
class Shooter(hwMap: HardwareMap) {

    companion object Params {
        @JvmField var kP = 0.005
        @JvmField var velocityThreshold: Double = 30.0
    }
    val shootCounterLeft = ShootCounter(150.0)
    val shootCounterRight = ShootCounter(150.0)
    val motorLeft: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterLeft"), false, 0.02)
    val motorRight: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterRight"), false, 0.02)

    val hoodServoLeft: Servo = hwMap.get(Servo::class.java, "hoodServoLeft")
    val hoodServoRight: Servo = hwMap.get(Servo::class.java, "hoodServoRight")

    var targetVelocityLeft = 0.0
    var targetVelocityRight = 0.0
    val velocityToPowerLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0),
        Pair(0.0001, 0.08),
        Pair(1380.0, 0.5),
        Pair(1500.0, 0.55),
        Pair(1640.0, 0.6),
        Pair(1800.0, 0.66),
        Pair(2150.0, 0.7779296875),
    ))



    init {
        motorLeft.mode = RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = RunMode.RUN_WITHOUT_ENCODER
        motorLeft.direction = DcMotorSimple.Direction.REVERSE
        motorRight.direction = DcMotorSimple.Direction.FORWARD
    }

    fun update() {
        shootCounterLeft.update(motorLeft.velocity, targetVelocityLeft)
        shootCounterRight.update(motorRight.velocity, targetVelocityRight)
        if (targetVelocityLeft == 0.0) {
            motorLeft.power = 0.0
        } else {
            motorLeft.power  = velocityToPowerLUT.get(targetVelocityLeft) + (targetVelocityLeft - motorLeft.velocity) * kP
        }

        if (targetVelocityRight == 0.0) {
            motorRight.power = 0.0
        } else {
            motorRight.power = velocityToPowerLUT.get(targetVelocityRight) + (targetVelocityRight - motorRight.velocity) * kP
        }
    }

    fun setHoodAngles(position: Double){
        hoodServoLeft.position = 1.0 - position
        hoodServoRight.position = position
    }

    fun setTargetVelocities(left: Double, right: Double = left){
        targetVelocityLeft = left
        targetVelocityRight = right
    }

    fun setTargetVelocityFromDistance(distance: Double) {
        setTargetVelocities(distanceToVelocityLeftLUT.get(distance), distanceToVelocityRightLUT.get(distance))
    }

    fun setHoodAngleFromDistance(distance: Double) {
        setHoodAngles(distanceToAngleLUT.get(distance))
    }

    fun setFirstHoodAngleAndVelocityFromDistance(distance: Double) {
        setTargetVelocities(secondaryDistanceToVelocityLeftLUT.get(distance), distanceToVelocityRightLUT.get(distance))
        setHoodAngles(distanceToAngleLUT.get(distance))
    }
    fun setSecondHoodAngleAndVelocityFromDistance(distance: Double) {
        setTargetVelocities(secondaryDistanceToVelocityLeftLUT.get(distance), secondaryDistanceToVelocityLeftLUT.get(distance))
        setHoodAngles(secondaryDistanceToAngleLUT.get(distance))
    }


    fun setHoodAngleAndVelocityFromDistance(distance: Double) {
        setTargetVelocityFromDistance(distance)
        setHoodAngleFromDistance(distance)
    }

    fun stop(): Command = Instant({
        targetVelocityRight = 0.0
        targetVelocityLeft = 0.0
    }, "Shooter:SpinDown")

    fun velocitiesInThreshold(threshold: Double = velocityThreshold) =
        abs(targetVelocityLeft - motorLeft.velocity) <= threshold
        && abs((targetVelocityRight) - motorRight.velocity) <= threshold

    fun waitForRightVelocity(): Command = WaitUntil {
        abs(targetVelocityRight - motorRight.velocity) <= velocityThreshold
    }

    fun waitForLeftVelocity(): Command = WaitUntil {
        abs(targetVelocityLeft - motorLeft.velocity) <= velocityThreshold
    }
    fun waitForVelocity(): Command = WaitUntil ({velocitiesInThreshold()}, "WaitForVelocity")
}