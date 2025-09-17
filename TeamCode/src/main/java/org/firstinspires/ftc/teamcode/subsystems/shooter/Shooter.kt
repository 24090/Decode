package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant

@Config
class Shooter(hwMap: HardwareMap) {
    val motor: DcMotor = hwMap.get(DcMotor::class.java, "shooter")
    var power = 0.0;

    init {
        motor.mode = RunMode.RUN_USING_ENCODER
    }
    companion object Params {
        var shootPower = 0.0
    }

    fun update() {
        motor.power = power;
    }

    fun spinUp(): Command = Instant({
        power = shootPower
    }, "SpinUp")

    fun spinDown(): Command = Instant({
        power = 0.0
    }, "SpinDown")
}