package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.DerivativeCalculator
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.InterpolatedLUT
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.SquID
import kotlin.math.abs
import kotlin.math.sqrt

@Config
class Shooter(hwMap: HardwareMap) {

    companion object Params {
        @JvmField var closeShootVelocity = 900.0
        @JvmField var farShootVelocity = 1225.0
        @JvmField var kP = 0.0025
    }

     val motorLeft: CachingDcMotorEx = CachingDcMotorEx(hwMap.get(DcMotorEx::class.java, "shooterLeft"))
     val motorRight: CachingDcMotorEx = CachingDcMotorEx(hwMap.get(DcMotorEx::class.java, "shooterRight"))
    var targetVelocity = 0.0;

    val velocityToPowerLUT = InterpolatedLUT(mapOf(
        Pair(0.0, 0.0),
        Pair(0.0001, 0.08),
        Pair(630.0, 0.28),
        Pair(950.0, 0.4),
        Pair(1650.0, 0.7),
    ))

    val distanceToVelocityLUT = InterpolatedLUT(mapOf(
        Pair(48 * sqrt(2.0), 930.0),
        Pair(72 * sqrt(2.0), 950.0),
        Pair(96 * sqrt(2.0), 1050.0),
        Pair(108 * sqrt(2.0), 1075.0)
    ))

    init {
        motorLeft.mode = RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = RunMode.RUN_WITHOUT_ENCODER
        motorLeft.direction = DcMotorSimple.Direction.REVERSE
        motorRight.direction = DcMotorSimple.Direction.FORWARD
    }

    fun update() {
        motorLeft.power  = velocityToPowerLUT.get(targetVelocity) + (targetVelocity - motorLeft.velocity) * kP
        motorRight.power = velocityToPowerLUT.get(targetVelocity) + (targetVelocity - motorRight.velocity) * kP
    }

    fun setTargetVelocityFromDistance(distance: Double) {
        targetVelocity = distanceToVelocityLUT.get(distance)
    }

    fun spinDown(): Command = Instant({
        targetVelocity = 0.0;
    }, "Shooter:SpinDown")

    fun waitForRightVelocity(): Command = WaitUntil {
        abs(targetVelocity - motorLeft.velocity) < 25.0
    }

    fun waitForLeftVelocity(): Command = WaitUntil {
        abs(targetVelocity - motorRight.velocity) < 25.0
    }
    fun waitForVelocity(): Command = WaitUntil {
        abs(targetVelocity - motorLeft.velocity) < 25.0
        && abs(targetVelocity - motorRight.velocity) < 25.0
    }
}

@TeleOp
@Config
class velocityToPowerTuner(): LinearOpMode(){
    companion object {
        @JvmField var targetVelocity = 1500.0
    }
    override fun runOpMode() {
        val shooter: Shooter = Shooter(hardwareMap)
        val dash = FtcDashboard.getInstance()
        val p = TelemetryPacket()
        p.put("leftVelocity", 0.0)
        p.put("rightVelocity", 0.0)
        p.put("targetVelocity", 1500.0)
        p.put("powerFeedforward", 0.0)
        dash.sendTelemetryPacket(p)
        waitForStart()
        while (opModeIsActive()){
            if (gamepad1.xWasPressed()) {
                targetVelocity += 50
            }
            if (gamepad1.yWasPressed()) {
                targetVelocity -= 50
            }
            shooter.update()
            shooter.targetVelocity = targetVelocity
            val p = TelemetryPacket()
            p.put("leftVelocity", shooter.motorLeft.velocity)
            p.put("rightVelocity", shooter.motorRight.velocity)
            p.put("powerFeedforward", shooter.velocityToPowerLUT.get(shooter.targetVelocity))
            dash.sendTelemetryPacket(p)
        }
    }

}