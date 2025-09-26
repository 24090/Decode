package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.farDistance
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.InterpolatedLUT
import kotlin.math.abs

@Config
class Shooter(hwMap: HardwareMap) {

    companion object Params {
        @JvmField var closeShootVelocity = 900.0
        @JvmField var farShootVelocity = 1225.0
    }

    private val motorLeft: CachingDcMotorEx = CachingDcMotorEx(hwMap.get(DcMotorEx::class.java, "shooterLeft"))
    private val motorRight: CachingDcMotorEx = CachingDcMotorEx(hwMap.get(DcMotorEx::class.java, "shooterRight"))
    var velocity = 0.0;

    val currentVelocity
        get() = (motorRight.getVelocity() + motorLeft.getVelocity())/2

    val velocityLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0),
        Pair(closeDistance, closeShootVelocity),
        Pair(farDistance, farShootVelocity)
    ))

    init {
        motorLeft.mode = RunMode.RUN_USING_ENCODER
        motorRight.mode = RunMode.RUN_USING_ENCODER
        motorRight.direction = DcMotorSimple.Direction.REVERSE
        motorLeft.setVelocityPIDFCoefficients(20.0,1.0,0.0,15.0)
        motorRight.setVelocityPIDFCoefficients(20.0,1.0,0.0,15.0)
    }

    fun update() {
        motorRight.velocity = velocity
        motorLeft.velocity = velocity
    }

    fun setTargetVelocityFromDistance(distance: Double) {
        velocity = velocityLUT.get(distance)
    }

    fun spinDown(): Command = Instant({
        velocity = 0.0;
    }, "Shooter:SpinDown")

    fun waitForVelocity(): Command = WaitUntil { abs(velocity - currentVelocity) < 25.0 }
}