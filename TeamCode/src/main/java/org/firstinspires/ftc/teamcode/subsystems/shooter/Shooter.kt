package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.InterpolatedLUT
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import kotlin.math.abs
import kotlin.math.sqrt

@Config
class Shooter(hwMap: HardwareMap) {

    companion object Params {
        @JvmField var kP = 0.005
        @JvmField var velocityThreshold: Double = 30.0
    }

    val motorLeft: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterLeft"), true, 0.02)
    val motorRight: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterRight"), true, 0.02)
    var targetVelocityLeft = 0.0
    var targetVelocityRight = 0.0
    val velocityToPowerLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0),
        Pair(0.0001, 0.08),
        Pair(1340.0, 0.5),
        Pair(1690.0, 0.6),
        Pair(2000.0, 0.7779296875),
    ))

    val distanceToVelocityLUT = InterpolatedLUT(mapOf(
        Pair(48 * sqrt(2.0), 1380.0),
        Pair(74.5 * sqrt(2.0), 1490.0),
        Pair(98.5 * sqrt(2.0), 1720.0),
        Pair(110.5 * sqrt(2.0), 1810.0)
    ))

    val exitVelocityToLeftVelocityLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0), // 0 in
        Pair(186.74179161227147, 1285.0), // 36 sqrt 2 in
        Pair(198.22446004746948, 1280.0), // 48 sqrt 2 in
        Pair(211.98762969131045, 1350.0),  // 60 sqrt 2 in
        Pair(225.90458755348956, 1470.0),  // 72 sqrt 2 in
        Pair(239.459044216468, 1570.0),  // 84 sqrt 2 in
        Pair(252.5208774351413, 1730.0),  // 96 sqrt 2 in
        Pair(265.07623836747837, 1880.0),  // 108 sqrt 2 in
    ))

    val exitVelocityToRightVelocityLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0), // 0 in
        Pair(186.74179161227147, 1405.0), // 36 sqrt 2 in
        Pair(198.22446004746948, 1390.0), // 48 sqrt 2 in
        Pair(211.98762969131045, 1450.0),  // 60 sqrt 2 in
        Pair(225.90458755348956, 1550.0),  // 72 sqrt 2 in
        Pair(239.459044216468, 1720.0),  // 84 sqrt 2 in
        Pair(252.5208774351413, 1815.0),  // 96 sqrt 2 in
        Pair(265.07623836747837, 1920.0),  // 108 sqrt 2 in
    ))

    init {
        motorLeft.mode = RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = RunMode.RUN_WITHOUT_ENCODER
        motorLeft.direction = DcMotorSimple.Direction.REVERSE
        motorRight.direction = DcMotorSimple.Direction.FORWARD
    }

    fun update() {
        motorLeft.power  = velocityToPowerLUT.get(targetVelocityLeft) + (targetVelocityLeft - motorLeft.velocity) * kP
        motorRight.power = velocityToPowerLUT.get(targetVelocityRight) + (targetVelocityRight - motorRight.velocity) * kP
    }

    fun setTargetVelocityFromDistance(distance: Double) {
        targetVelocityLeft = distanceToVelocityLUT.get(distance)
        targetVelocityRight = distanceToVelocityLUT.get(distance) + 50.0
    }

    fun stop(): Command = Instant({
        targetVelocityRight = 0.0
        targetVelocityLeft = 0.0
    }, "Shooter:SpinDown")

    fun waitForRightVelocity(): Command = WaitUntil {
        abs(targetVelocityRight - motorRight.velocity) <= velocityThreshold
    }

    fun waitForLeftVelocity(): Command = WaitUntil {
        abs(targetVelocityLeft - motorLeft.velocity) <= velocityThreshold
    }
    fun waitForVelocity(): Command = WaitUntil ({
        abs(targetVelocityLeft - motorLeft.velocity) <= velocityThreshold
        && abs((targetVelocityRight) - motorRight.velocity) <= velocityThreshold
    }, "WaitForVelocity")
}