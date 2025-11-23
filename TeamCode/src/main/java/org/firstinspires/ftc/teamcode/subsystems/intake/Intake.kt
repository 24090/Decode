package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.config.Config
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
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.VoltageCompensatedMotor
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.util.BallColor
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.clamp
import java.util.Optional
import kotlin.math.abs
import kotlin.math.sign

@Config
class Intake(hwMap: HardwareMap) {
    val motor: VoltageCompensatedMotor = VoltageCompensatedMotor(hwMap.get(DcMotorEx::class.java, "intake"), false, 0.02)
    val pusherLeft: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherLeft"))
    val pusherRight: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherRight"))
    val huskyLens: HuskyLens = HuskyLens(hwMap)
    val indexTracker: IndexTracker = IndexTracker()

    var behaviour: IntakeBehaviour = IntakeBehaviour.Velocity(0.0)
    init {
        motor.mode = RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    sealed class IntakeBehaviour(var target: Double) {
        class Velocity(target: Double): IntakeBehaviour(target)
        class Position(target: Int): IntakeBehaviour(target.toDouble())
    }
    companion object Params {
        @JvmField var runVelocity = 1000.0
        @JvmField var kF = 0.5/1000
        @JvmField var kP = 0.001
        @JvmField var powerMax = 0.5
        @JvmField var pusherLeftForward = 0.0
        @JvmField var pusherLeftBack = 0.53

        @JvmField var pusherRightForward = 0.6
        @JvmField var pusherRightBack = 0.07
        @JvmField var pusherWait = 0.5
        @JvmField var adjustDistance = 45

        @JvmField var kP_pos = 0.02
        @JvmField var kD_Pos = 0.001
        @JvmField var kL_Pos = 0.15
        @JvmField var kT_Pos = 4.0
    }

    fun update() {
        motor.power = when (behaviour) {
            is IntakeBehaviour.Position ->
                PDLT((behaviour.target - motor.currentPosition), -motor.velocity, kP_pos, kD_Pos, kL_Pos, kT_Pos)
            is IntakeBehaviour.Velocity ->
                clamp(behaviour.target * kF + (behaviour.target - motor.velocity) * kP, -1.0, powerMax)
        }
    }

    fun spinUp(): Command = Instant({
        behaviour = IntakeBehaviour.Velocity(runVelocity)
    }, "SpinUp")

    fun setAdjustThird(): Command = Instant({
        behaviour = IntakeBehaviour.Position(motor.currentPosition - adjustDistance)
    }, "setAdjustThird")

    fun fullAdjustThird(): Command = Sequence(
        setAdjustThird(),
        WaitUntil { behaviour.target >= motor.currentPosition },
        name = "FullAdjustThird"
    )
    fun stop(): Command = Instant({
        behaviour = IntakeBehaviour.Velocity(0.0)
    }, "StopIntake")

    fun releaseDual(): Command = Parallel(
        releaseLeft(),
        releaseRight(),
        name = "ReleaseDual"
    )

    fun waitForStall(): Command =
        WaitUntil({
            if (behaviour is IntakeBehaviour.Velocity){
                abs(motor.velocity) < 100
            } else {
                true
            }
        }, "waitForIntakeVelocity")

    fun waitForDistance(distance: Int): Command = Future{
        val target = motor.currentPosition + distance
        val sign = sign((target - motor.currentPosition).toDouble())
        WaitUntil({
           sign((target - motor.currentPosition).toDouble()) != sign
        }, "waitForIntakeVelocity")
    }
    fun waitForVelocity(): Command = WaitUntil({
            if (behaviour is IntakeBehaviour.Velocity){
                abs(motor.velocity - behaviour.target) < 30
            } else {
                true
            }
        }, "waitForIntakeVelocity"
        )


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

    fun releaseReccomended(): Command = Instant{
        huskyLens.read()
        val currentReccomendation: Pair<Optional<BallColor>, Optional<BallColor>> = Pair<Optional<BallColor>,Optional<BallColor>>(Optional.of(indexTracker.getRecommendations().first), Optional.of(indexTracker.getRecommendations().second))
        val left: Optional<BallColor> = huskyLens.getColorPair().first
        val right: Optional<BallColor> = huskyLens.getColorPair().second
        Sequence(Instant{
            if ((left == right) && (left == currentReccomendation.first) && (currentReccomendation.first == currentReccomendation.second)){
                releaseDual()
            }
            if ((left == right) && !(left == currentReccomendation.first)){
                releaseLeft()
            }
            if (!(left==right) && (left == currentReccomendation.first)){
                releaseLeft()
            }
            if (!(left==right) && (right == currentReccomendation.first)){
                releaseRight()
            }
        }, Instant{
            huskyLens.read()
            if ((left == right) && (left == currentReccomendation.first) && (currentReccomendation.first == currentReccomendation.second)){
                releaseDual()
            }
            if ((left == right) && !(left == currentReccomendation.first)){
                Sequence(releaseLeft(),releaseRight())
            }
            if (!(left==right) && (left == currentReccomendation.first)){
                Sequence(releaseLeft(),releaseRight())
            }
            if (!(left==right) && (right == currentReccomendation.first)){
                Sequence(releaseRight(),releaseLeft())
            }
        }
        )

    }
}