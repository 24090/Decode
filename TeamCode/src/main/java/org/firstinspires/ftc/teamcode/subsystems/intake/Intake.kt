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
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.abs

@Config
class Intake(hwMap: HardwareMap) {
    val motor: CachingDcMotorEx = CachingDcMotorEx(hwMap.get(DcMotorEx::class.java, "intake"))
    val pusherLeft: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherLeft"))
    val pusherRight: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherRight"))

    var behaviour: IntakeBehaviour = IntakeBehaviour.Velocity(0.0)
    init {
        motor.mode = RunMode.RUN_USING_ENCODER
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    sealed class IntakeBehaviour(var target: Double) {
        class Velocity(target: Double): IntakeBehaviour(target)
        class Position(target: Int): IntakeBehaviour(target.toDouble())
    }
    companion object Params {
        @JvmField var runVelocity = 600.0
        @JvmField var kF = 0.2/480
        @JvmField var kP = 0.005
        @JvmField var powerMax = 0.5
        @JvmField var pusherLeftForward = 0.0
        @JvmField var pusherLeftBack = 0.45

        @JvmField var pusherRightForward = 0.67
        @JvmField var pusherRightBack = 0.12
        @JvmField var pusherWait = 0.5
    }

    fun update() {
        motor.power = when (behaviour) {
            is IntakeBehaviour.Position ->
                PDLT((behaviour.target - motor.currentPosition), -motor.velocity, 0.05, 0.012, 0.7, 0.0)
            is IntakeBehaviour.Velocity ->
                clamp(behaviour.target * kF + (behaviour.target - motor.velocity) * kP, -1.0, powerMax)
        }
    }

    fun spinUp(): Command = Instant({
        behaviour = IntakeBehaviour.Velocity(runVelocity)
    }, "SpinUp")

    fun spinReverse(): Command = Instant({
        behaviour = IntakeBehaviour.Velocity(-300.0)
    }, "SpinUp")

    fun setAdjustThird(): Command = Instant({
        behaviour = IntakeBehaviour.Position(motor.currentPosition - 21)
    }, "setAdjustThird")

    fun fullAdjustThird(): Command = Sequence(
        setAdjustThird(),
        WaitUntil { behaviour.target >= motor.currentPosition }
    )
    fun stop(): Command = Instant({
        behaviour = IntakeBehaviour.Velocity(0.0)
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
        @JvmField var targetVelocity = 0.0
    }
    override fun runOpMode() {
        val intake: Intake = Intake(hardwareMap)
        val shooter: Shooter = Shooter(hardwareMap)
        val dash = FtcDashboard.getInstance()
        val p = TelemetryPacket()
        val f = {
            intake.update()
            intake.behaviour.target = targetVelocity
            val p = TelemetryPacket()
            p.put("velocity", intake.motor.velocity)
            p.put("targetVelocity", intake.behaviour.target)
            p.put("powerFeedforward", Intake.kF * intake.behaviour.target)
            dash.sendTelemetryPacket(p)
        }
        p.put("velocity", 0.0)
        p.put("targetVelocity", 1500.0)
        p.put("powerFeedforward", 0.0)
        dash.sendTelemetryPacket(p)
        waitForStart()
        shooter.motorLeft.power = 0.12
        shooter.motorRight.power = 0.12
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
            if (gamepad1.xWasPressed()){
                runBlocking(Race(
                    Forever(intake::update),
                    Sequence(
                        intake.fullAdjustThird(),
                        intake.stop()
                    )

                ))
            }
            f()

        }
    }

}