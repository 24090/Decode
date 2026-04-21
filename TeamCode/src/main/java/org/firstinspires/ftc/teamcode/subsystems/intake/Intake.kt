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
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PD
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDL
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PV
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.StallTest
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.abs
import kotlin.math.absoluteValue

@Config
class Intake(hwMap: HardwareMap) {
    val motor: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "intake"), false, 0.02)
    val motorBack: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "intakeBack"), false, 0.02)
    val pusherLeft: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherLeft"))
    val pusherRight: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherRight"))

    var behaviour: IntakeBehaviour = IntakeBehaviour.Stop
    var stopPosition: Int? = null
    val stallTest: StallTest = StallTest(40)
    fun isStalling() = (stallTest.get() < 1050) && (stallTest.deriv().absoluteValue < 100) && (stallTest.get() > 500)
    private var nextShootTime: Long? = null

    init {
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
        motorBack.direction = DcMotorSimple.Direction.FORWARD
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    sealed class IntakeBehaviour() {
        object Greedy: IntakeBehaviour()
        object Grab: IntakeBehaviour()
        object Hold: IntakeBehaviour()
        object Eject: IntakeBehaviour()
        object Stop: IntakeBehaviour()
        class Wheelie(val fl: DcMotor, val fr: DcMotor): IntakeBehaviour()
    }
    companion object Params {
        @JvmField var runVelocity = 1100.0
        @JvmField var runVelocityBack = 1100.0

        @JvmField var backF = 0.00037
        @JvmField var frontF = 0.00051
        @JvmField var kP = 0.0003
        @JvmField var kP_positional = 0.01
        @JvmField var kD_positional = 0.0002
        @JvmField var kL_positional = 0.15



        @JvmField var powerMax = 0.67
        @JvmField var pusherLeftForward = 0.0
        @JvmField var pusherLeftBack = 0.95

        @JvmField var pusherRightForward = 0.6
        @JvmField var pusherRightBack = 0.0
        @JvmField var pusherWait = 0.05
    }

    private fun holdFront() {
        if (abs(motor.velocity) > 40 && stopPosition == null) {
            motor.power = clamp(-motor.velocity * kP, -powerMax, powerMax)
        } else if (stopPosition != null){
            motor.power = clamp(PDL((stopPosition!! - motor.currentPosition).toDouble(), motor.velocity, kP_positional, kD_positional, kL_positional), -powerMax, powerMax)
        } else {
            stopPosition = motor.currentPosition
        }
    }
    private fun runFront() {
        stopPosition = null
        motor.power = clamp(PV(runVelocity, motor.velocity, kP, frontF), -1.0, powerMax)
    }
    private fun runBack() { motorBack.power = clamp(PV(runVelocity, motorBack.velocity, kP, backF), -1.0, powerMax) }

    fun update() {
        val behaviour = behaviour
        stallTest.update(motorBack.velocity.toInt(), System.currentTimeMillis())
        when (behaviour) {
            IntakeBehaviour.Hold -> {
                runBack()
                holdFront()
            }
            IntakeBehaviour.Grab -> {
                runBack()
                if (!isStalling()) {
                    runFront()
                } else {
                    holdFront()
                }

            }
            IntakeBehaviour.Greedy -> {
                runBack()
                runFront()
            }
            IntakeBehaviour.Stop -> {
                holdFront()
                motorBack.power = clamp((0 -motor.velocity) * kP, -1.0, powerMax)
            }
            IntakeBehaviour.Eject -> {
                motorBack.power = clamp(-runVelocityBack * backF + (-runVelocityBack - motorBack.velocity) * kP, -1.0, powerMax)
                motor.power =  clamp(-runVelocity * frontF + (-runVelocity - motor.velocity) * kP, -powerMax, 1.0)
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
            pusherWait
        } else {
            (nextShootTime - System.currentTimeMillis())/1000.0
        }
    }

    fun releaseDual(): Command = Parallel(
        Instant { nextShootTime = System.currentTimeMillis() + (pusherWait * 1000).toLong()},
        releaseLeft(),
        releaseRight(),
        Instant { nextShootTime = null},
        name = "ReleaseDual"
    )
    fun waitForStall(): Command =
        WaitUntil({
            isStalling()
        }, "waitForIntakeVelocity")

    fun resetPushers(){
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    fun releaseLeft(): Command = Sequence(
        Instant { pusherLeft.position = pusherLeftForward },
        Sleep(pusherWait),
        Instant { pusherLeft.position = pusherLeftBack },

        name = "ReleaseLeft"
    )

    fun releaseRight(): Command = Sequence(
        Instant { pusherRight.position = pusherRightForward },
        Sleep(pusherWait),
        Instant { pusherRight.position = pusherRightBack },
        name = "ReleaseRight"
    )
}