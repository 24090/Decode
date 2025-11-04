package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingCRServo
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import org.firstinspires.ftc.teamcode.commands.*

@Config
class Intake(hwMap: HardwareMap) {
    val motor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java, "intake"))
    val pusherLeft: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherLeft"))
    val pusherRight: CachingServo = CachingServo(hwMap.get(Servo::class.java, "pusherRight"))
    var power = 0.0;

    init {
        motor.mode = RunMode.RUN_USING_ENCODER
        pusherLeft.position = pusherLeftBack
        pusherRight.position = pusherRightBack
    }
    companion object Params {
        @JvmField var runPower = 0.2
        @JvmField var pusherLeftForward = 0.0
        @JvmField var pusherLeftBack = 0.54

        @JvmField var pusherRightForward = 0.67
        @JvmField var pusherRightBack = 0.0
        @JvmField var pusherWait = 0.5
    }

    fun update() {
        motor.power = power;
    }

    fun spinUp(): Command = Instant({
        power = runPower
    }, "SpinUp")

    fun spinDown(): Command = Instant({
        power = 0.2
    }, "SpinDown")

    fun releaseDual(): Command = Parallel(
        releaseLeft(),
        releaseRight()
    )
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