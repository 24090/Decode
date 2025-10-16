package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingCRServo
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor
import org.firstinspires.ftc.teamcode.commands.*

@Config
class Intake(hwMap: HardwareMap) {
    val motor: CachingDcMotor = CachingDcMotor(hwMap.get(DcMotor::class.java, "intake"))
    val pusherLeft: CachingCRServo = CachingCRServo(hwMap.get(CRServo::class.java, "pusherLeft"))
    val pusherRight: CachingCRServo = CachingCRServo(hwMap.get(CRServo::class.java, "pusherRight"))
    var power = 0.0;

    init {
        motor.mode = RunMode.RUN_USING_ENCODER
        pusherLeft.power = 0.0
        pusherRight.power = 0.0
    }
    companion object Params {
        @JvmField var runPower = 1.0
        @JvmField var pusherForward = -1.0
        @JvmField var pusherBack = 1.0
        @JvmField var pusherWait = 1.0
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
        Instant({ pusherLeft.power = pusherForward}),
        Sleep(pusherWait),
        Instant({ pusherLeft.power = pusherBack}),
        name = "ReleaseLeft"
    )

    fun releaseRight(): Command = Sequence(
        Instant({ pusherRight.power = pusherForward}),
        Sleep(pusherWait),
        Instant({ pusherRight.power = pusherBack}),
        name = "ReleaseRight"
    )
}