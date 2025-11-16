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
        @JvmField var rightVelocityOffset: Double = 30.0
        @JvmField var velocityThreshold: Double = 30.0
    }

    val motorLeft: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterLeft"), true, 0.02)
    val motorRight: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "shooterRight"), true, 0.02)
    var targetVelocity = 0.0

    val velocityToPowerLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0),
        Pair(0.0001, 0.08),
        Pair(920.0, 0.43),
        Pair(1560.0, 0.7),
        Pair(1800.0, 0.95),
        Pair(2200.0, 0.95),
    ))

    val distanceToVelocityLUT = InterpolatedLUT(mapOf(
        Pair(48 * sqrt(2.0), 1530.0),
        Pair(72 * sqrt(2.0), 1700.0),
        Pair(96 * sqrt(2.0), 1800.0),
        Pair(108 * sqrt(2.0), 2025.0)
    ))

    init {
        motorLeft.mode = RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = RunMode.RUN_WITHOUT_ENCODER
        motorLeft.direction = DcMotorSimple.Direction.REVERSE
        motorRight.direction = DcMotorSimple.Direction.FORWARD
    }

    fun update() {
        motorLeft.power  = velocityToPowerLUT.get(targetVelocity) + (targetVelocity - motorLeft.velocity) * kP
        motorRight.power = velocityToPowerLUT.get(targetVelocity + rightVelocityOffset) + (targetVelocity + rightVelocityOffset - motorRight.velocity) * kP
    }

    fun setTargetVelocityFromDistance(distance: Double) {
        targetVelocity = distanceToVelocityLUT.get(distance)
    }

    fun stop(): Command = Instant({
        targetVelocity = 0.0
    }, "Shooter:SpinDown")

    fun waitForRightVelocity(): Command = WaitUntil {
        abs(targetVelocity - motorLeft.velocity) <= velocityThreshold
    }

    fun waitForLeftVelocity(): Command = WaitUntil {
        abs(targetVelocity - motorRight.velocity) <= velocityThreshold
    }
    fun waitForVelocity(): Command = WaitUntil ({
        abs(targetVelocity - motorLeft.velocity) <= velocityThreshold
        && abs((targetVelocity + rightVelocityOffset) - motorRight.velocity) <= velocityThreshold
    }, "WaitForVelocity")
}