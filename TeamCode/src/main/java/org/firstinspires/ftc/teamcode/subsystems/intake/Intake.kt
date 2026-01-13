package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.util.BallColor
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Pattern
import org.firstinspires.ftc.teamcode.util.clamp
import java.util.Optional
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.sign

@Config
class Intake(hwMap: HardwareMap) {
    val motor: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "intake"), false, 0.02)
    val motorBack: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "intakeBack"), false, 0.02)
    val pusherLeft: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherLeft"))
    val pusherRight: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherRight"))

    var behaviour: IntakeBehaviour = IntakeBehaviour.Stop

    private var nextShootTime: Long? = null

    init {
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
        motorBack.direction = DcMotorSimple.Direction.FORWARD
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    sealed class IntakeBehaviour() {
        object Grab: IntakeBehaviour()
        object Hold: IntakeBehaviour()
        object Stop: IntakeBehaviour()
        object Stopfront: IntakeBehaviour()
        class Wheelie(val fl: DcMotor, val fr: DcMotor): IntakeBehaviour()
    }
    companion object Params {
        @JvmField var runVelocity = 800.0

        @JvmField var backF = 0.00045
        @JvmField var frontF = 0.00051
        @JvmField var kP = 0.0003
        @JvmField var powerMax = 0.5
        @JvmField var pusherLeftForward = 0.15
        @JvmField var pusherLeftBack = 0.53

        @JvmField var pusherRightForward = 0.6
        @JvmField var pusherRightBack = 0.00
        @JvmField var pusherWait = 0.4

        @JvmField var kP_pos = 0.02
        @JvmField var kD_Pos = 0.001
        @JvmField var kL_Pos = 0.15
        @JvmField var kT_Pos = 3.0
    }

    fun update() {
        val behaviour = behaviour
        when (behaviour) {
            IntakeBehaviour.Hold -> {
                motor.power =
                    clamp(runVelocity * frontF + (runVelocity - motor.velocity) * kP, -1.0, powerMax)
                motorBack.power = clamp(-runVelocity * backF + (-runVelocity - motorBack.velocity) * kP, -1.0, powerMax)
            }
            IntakeBehaviour.Grab -> {
                motorBack.power = clamp(runVelocity * backF + (runVelocity - motorBack.velocity) * kP, -1.0, powerMax)
                if (motorBack.velocity > 600) {
                    motor.power = clamp(runVelocity * frontF + (runVelocity - motor.velocity) * kP, -1.0, powerMax)
                } else {
                    motor.power = clamp(-400 * frontF + (-400 - motor.velocity) * kP, -powerMax, 1.0)
                }

            }
            IntakeBehaviour.Stop -> {
                motor.power = clamp((0 - motor.velocity) * kP, -1.0, powerMax)
                motorBack.power = clamp((0 -motor.velocity) * kP, -1.0, powerMax)
            }
            IntakeBehaviour.Stopfront -> {
                motor.power = clamp((0 - motor.velocity) * kP, -1.0, powerMax)
                motorBack.power = clamp(runVelocity * backF + (runVelocity - motorBack.velocity) * kP, -1.0, powerMax)
            }
            is IntakeBehaviour.Wheelie -> {
                motor.power = -(behaviour.fl.power + behaviour.fr.power)/10.0
            }
        }
    }

    fun spinUp(): Command = Instant({
        behaviour = IntakeBehaviour.Grab
    }, "SpinUp")

    fun setAdjustThird(): Command = Instant({
        behaviour = IntakeBehaviour.Hold
    }, "SetAdjustThird")

    fun stopFront(): Command = Instant({
        behaviour = IntakeBehaviour.Stopfront
    }, "StopFront")

    fun fullAdjustThird(): Command = Sequence(
        setAdjustThird(),
        Future {
            val ticks = motorBack.currentPosition;
            return@Future WaitUntil { motorBack.currentPosition < ticks - 50 }
        },
        name = "FullAdjustThird"
    )
    fun stop(): Command = Instant({
        behaviour = IntakeBehaviour.Stop
    }, "StopIntake")

    fun getMinShootTime(): Double {
        val nextShootTime = nextShootTime
        return if (nextShootTime == null){
            0.3
        } else {
            (nextShootTime - System.currentTimeMillis())/1000.0
        }
    }

    fun releaseDual(): Command = Parallel(
        Instant { nextShootTime = System.currentTimeMillis() + 300},
        releaseLeft(),
        releaseRight(),
        name = "ReleaseDual"
    )
    fun waitForStall(): Command =
        WaitUntil({
            if (behaviour == IntakeBehaviour.Grab){
                abs(motorBack.velocity) < 100
            } else {
                true
            }
        }, "waitForIntakeVelocity")

    fun resetPushers(){
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    fun releaseLeft(): Command = Sequence(
        Instant { pusherLeft.position = pusherLeftForward },
        Sleep(pusherWait),
        Instant {
            pusherLeft.position = pusherLeftBack
            nextShootTime = null
        },

        name = "ReleaseLeft"
    )

    fun releaseRight(): Command = Sequence(
        Instant { pusherRight.position = pusherRightForward },
        Sleep(pusherWait),
        Instant {
            pusherRight.position = pusherRightBack
            nextShootTime = null
        },
        name = "ReleaseRight"
    )
}