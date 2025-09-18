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
    val blockerServo = hwMap.get(Servo::class.java, "blockerServo")
    var power = 0.0;

    init {
        motor.mode = RunMode.RUN_USING_ENCODER
        blockerServo.position = blockerBlock
    }
    companion object Params {
        @JvmField var runPower = 1.0
        @JvmField var blockerBlock = 0.0
        @JvmField var blockerUnblock = 1.0
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
        Instant({blockerServo.position = blockerUnblock}),
        Sleep(2.0),
        Instant({blockerServo.position = blockerBlock}),
    )
}