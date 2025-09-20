package org.firstinspires.ftc.teamcode.subsystems.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commands.*

@Config
class Intake(hwMap: HardwareMap) {
    val motor: DcMotor = hwMap.get(DcMotor::class.java, "intake")
    val pusher: CRServo = hwMap.get(CRServo::class.java, "pusher")
    var power = 0.0;

    init {
        motor.mode = RunMode.RUN_USING_ENCODER
        pusher.power = pusherBack
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
        power = 0.0
    }, "SpinDown")

    fun releaseBall(): Command = Sequence(
        Instant({ pusher.power = pusherForward}),
        Sleep(pusherWait),
        Instant({ pusher.power = pusherBack}),
        name = "ReleaseBall"
    )
}