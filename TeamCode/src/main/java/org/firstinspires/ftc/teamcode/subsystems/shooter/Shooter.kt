package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant

@Config
class Shooter(hwMap: HardwareMap) {
    val motorLeft: DcMotor = hwMap.get(DcMotor::class.java, "shooterLeft")
    val motorRight: DcMotor = hwMap.get(DcMotor::class.java, "shooterRight")
    var power = 0.0;

    init {
        motorLeft.mode = RunMode.RUN_USING_ENCODER
        motorRight.mode = RunMode.RUN_USING_ENCODER
        motorRight.direction = DcMotorSimple.Direction.REVERSE
    }
    companion object Params {
        @JvmField var shootPower = 1.0
    }

    fun update() {
        motorLeft.power = power;
        motorRight.power = power;
    }

    fun spinUp(): Command = Instant({
        power = shootPower
    }, "SpinUp")

    fun spinDown(): Command = Instant({
        power = 0.0
    }, "SpinDown")
}