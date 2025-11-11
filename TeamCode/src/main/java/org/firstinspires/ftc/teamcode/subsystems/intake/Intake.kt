package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.clamp

@Config
class Intake(hwMap: HardwareMap) {
    val motor: CachingDcMotorEx = CachingDcMotorEx(hwMap.get(DcMotorEx::class.java, "intake"))
    val pusherLeft: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherLeft"))
    val pusherRight: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherRight"))
    var targetVelocity = 0.0;

    init {
        motor.mode = RunMode.RUN_USING_ENCODER
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    companion object Params {
        @JvmField var runVelocity = 500.0
        @JvmField var kF = 0.2/480
        @JvmField var kP = 0.005
        @JvmField var powerMax = 0.5
        @JvmField var pusherLeftForward = 0.0
        @JvmField var pusherLeftBack = 0.4

        @JvmField var pusherRightForward = 0.67
        @JvmField var pusherRightBack = 0.0
        @JvmField var pusherWait = 0.5
    }

    fun update() {
        motor.power = clamp(targetVelocity * kF + (targetVelocity - motor.velocity) * kP, -1.0, powerMax);
    }

    fun spinUp(): Command = Instant({
        targetVelocity = runVelocity
    }, "SpinUp")

    fun spinReverse(): Command = Instant({
        targetVelocity = -300.0
    }, "SpinUp")

    fun spinDown(): Command = Instant({
        targetVelocity = 0.0
    }, "SpinDown")

    fun releaseDual(): Command = Parallel(
        releaseLeft(),
        releaseRight()
    )

    fun resetPushers(){
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    fun releaseLeft(): Command = Sequence(
        Instant({ pusherLeft.position = pusherLeftForward}),
        Sleep(pusherWait),
        Instant({ pusherLeft.position = pusherLeftBack}),
        name = "ReleaseLeft"
    )

    fun releaseRight(): Command = Sequence(
        Instant({ pusherRight.position = pusherRightForward}),
        Sleep(pusherWait),
        Instant({ pusherRight.position = pusherRightBack}),
        name = "ReleaseRight"
    )
}

@TeleOp
@Config
class intakeTesting(): LinearOpMode(){
    companion object {
        @JvmField var targetVelocity = 1000.0
    }
    override fun runOpMode() {
        val intake: Intake = Intake(hardwareMap)
        val shooter: Shooter = Shooter(hardwareMap)
        val dash = FtcDashboard.getInstance()
        val p = TelemetryPacket()
        val f = {
            intake.update()
            intake.targetVelocity = targetVelocity
            val p = TelemetryPacket()
            p.put("velocity", intake.motor.velocity)
            p.put("targetVelocity", intake.targetVelocity)
            p.put("powerFeedforward", Intake.kF * intake.targetVelocity)
            dash.sendTelemetryPacket(p)
        }
        p.put("velocity", 0.0)
        p.put("targetVelocity", 1500.0)
        p.put("powerFeedforward", 0.0)
        dash.sendTelemetryPacket(p)
        waitForStart()
        shooter.motorLeft.power = 0.15
        shooter.motorRight.power = 0.15
        while (opModeIsActive()){
            if (gamepad1.leftBumperWasPressed()){
                runBlocking(Race(
                    Forever(f),
                    intake.releaseLeft()
                ))
            } 
            if (gamepad1.rightBumperWasPressed()){
                runBlocking(Race(
                    Forever(f),
                    intake.releaseRight()
                ))
            }
            f()

        }
    }

}