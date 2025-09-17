package org.firstinspires.ftc.teamcode.subsystems.intake

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant

class Intake(hwMap: HardwareMap) {
    val actuator: CRServo = hwMap.get(CRServo::class.java, "intake")
    var power = 0.0;
    companion object Params {
        var intakePower = 0.0
    }

    fun update() {
        actuator.power = power
    }

    fun spinUp(): Command = Instant({
        power = intakePower
    }, "SpinUp")

    fun spinDown(): Command = Instant({
        power = 0.0
    }, "SpinDown")
}