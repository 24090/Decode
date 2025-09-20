package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.DerivativeCalculator
import kotlin.math.abs

@Config
class Shooter(hwMap: HardwareMap) {
    private val motorLeft: DcMotorEx = hwMap.get(DcMotor::class.java, "shooterLeft") as DcMotorEx
    private val motorRight: DcMotorEx = hwMap.get(DcMotor::class.java, "shooterRight") as DcMotorEx
    private var velocity = 0.0;
    val currentVelocity
        get() = (motorRight.getVelocity() + motorLeft.getVelocity())/2
    init {
        motorLeft.mode = RunMode.RUN_USING_ENCODER
        motorRight.mode = RunMode.RUN_USING_ENCODER
        motorRight.direction = DcMotorSimple.Direction.REVERSE
        motorLeft.setVelocityPIDFCoefficients(20.0,0.0,0.0,15.0)
        motorRight.setVelocityPIDFCoefficients(20.0,0.0,0.0,15.0)
    }
    companion object Params {
        @JvmField var closeShootVelocity = 950.0
        @JvmField var farShootVelocity = 1275.0
    }

    fun update() {
        motorRight.velocity = velocity
        motorLeft.velocity = velocity
    }
    fun spinUpClose(): Command = Instant({
        velocity = closeShootVelocity
    }, "SpinUp")

    fun spinUpFar(): Command = Instant({
        velocity = farShootVelocity
    }, "SpinUp")

    fun spinDown(): Command = Instant({
        velocity = 0.0;
    }, "SpinDown")

    fun waitForSpeed(): Command = WaitUntil { abs(velocity - currentVelocity) < 25.0 }
}