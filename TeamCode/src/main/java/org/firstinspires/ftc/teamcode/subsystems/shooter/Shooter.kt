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

@Config
class Shooter(hwMap: HardwareMap) {

    companion object Params {
        @JvmField var kP = 0.005
        @JvmField var velocityThreshold: Double = 30.0
    }
    val shootCounterLeft = ShootCounter(150.0)
    val shootCounterRight = ShootCounter(150.0)
    val motorLeft: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterLeft"), true, 0.02)
    val motorRight: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterRight"), true, 0.02)

    val hoodServo: Servo = hwMap.get(Servo::class.java, "hoodServo")
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

    val distanceToVelocityLeftLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0), // 0 in
        Pair(36*sqrt(2.0), 1360.0 ), // 36 sqrt 2 in
        Pair(48*sqrt(2.0), 1340.0), // 48 sqrt 2 in
        Pair(60*sqrt(2.0), 1430.0),  // 60 sqrt 2 in
        Pair(72*sqrt(2.0), 1540.0),  // 72 sqrt 2 in
        Pair(84*sqrt(2.0), 1620.0),  // 84 sqrt 2 in
        Pair(96*sqrt(2.0), 1755.0),  // 96 sqrt 2 in
        Pair(108*sqrt(2.0), 1840.0),
    ))
    val distanceToVelocityRightLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0), // 0 in
        Pair(36*sqrt(2.0), 1360.0), // 36 sqrt 2 in
        Pair(48*sqrt(2.0), 1325.0), // 48 sqrt 2 in
        Pair(60*sqrt(2.0), 1430.0),  // 60 sqrt 2 in
        Pair(72*sqrt(2.0), 1525.0),  // 72 sqrt 2 in
        Pair(84*sqrt(2.0), 1620.0),  // 84 sqrt 2 in
        Pair(96*sqrt(2.0), 1774.0),  // 96 sqrt 2 in
        Pair(108*sqrt(2.0), 1940.0),
    ))

    val distanceToAngleLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0), // 0 in
        Pair(36*sqrt(2.0), 0.0), // 36 sqrt 2 in
        Pair(48*sqrt(2.0), 0.0), // 48 sqrt 2 in
        Pair(60*sqrt(2.0), 0.0),  // 60 sqrt 2 in
        Pair(72*sqrt(2.0), 0.0),  // 72 sqrt 2 in
        Pair(84*sqrt(2.0), 0.0),  // 84 sqrt 2 in
        Pair(96*sqrt(2.0), 0.0),  // 96 sqrt 2 in
        Pair(108*sqrt(2.0), 0.0),
    ))

    val exitVelocityToLeftVelocityLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0), // 0 in
        Pair(181.1681243438419, 1355.0), // 36 sqrt 2 in
        Pair(198.54371227569945, 1300.0), // 48 sqrt 2 in
        Pair(215.6597144491923, 1390.0),  // 60 sqrt 2 in
        Pair(231.97155294255114, 1520.0),  // 72 sqrt 2 in
        Pair(247.3880987428304, 1620.0),  // 84 sqrt 2 in
        Pair(262.0039126511864, 1730.0),  // 96 sqrt 2 in
        Pair(275.90206925170855, 1930.0),  // 108 sqrt 2 in
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

    fun setTargetVelocities(left: Double, right: Double = left){
        targetVelocityLeft = left
        targetVelocityRight = right
    }

    fun setTargetVelocityFromDistance(distance: Double) {
        setTargetVelocities(distanceToVelocityLeftLUT.get(distance), distanceToVelocityRightLUT.get(distance))
    }

    fun setHoodAngleFromDistance(distance: Double) {
        hoodServo.position = distanceToAngleLUT.get(distance)
    }

    fun setHoodAngleAndVelocityFromDistance(distance: Double) {
        setTargetVelocities(distanceToVelocityLeftLUT.get(distance), distanceToVelocityRightLUT.get(distance))
        hoodServo.position = distanceToAngleLUT.get(distance)
    }

    fun stop(): Command = Instant({
        targetVelocityRight = 0.0
        targetVelocityLeft = 0.0
    }, "Shooter:SpinDown")

    fun velocitiesInThreshold(threshold: Double = velocityThreshold) =
        abs(targetVelocityLeft - motorLeft.velocity) <= velocityThreshold
        && abs((targetVelocityRight) - motorRight.velocity) <= velocityThreshold

    fun waitForRightVelocity(): Command = WaitUntil {
        abs(targetVelocityRight - motorRight.velocity) <= velocityThreshold
    }

    fun waitForLeftVelocity(): Command = WaitUntil {
        abs(targetVelocityLeft - motorLeft.velocity) <= velocityThreshold
    }
    fun waitForVelocity(): Command = WaitUntil ({velocitiesInThreshold(velocityThreshold)}, "WaitForVelocity")
}