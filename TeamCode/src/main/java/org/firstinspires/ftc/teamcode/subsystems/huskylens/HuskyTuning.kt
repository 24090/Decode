package org.firstinspires.ftc.teamcode.subsystems.huskylens

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class HuskyTuning(): LinearOpMode(){
    override fun runOpMode() {
        waitForStart()
        var huskyLens: HuskyLens = HuskyLens(hardwareMap)
        while (opModeIsActive()){
            huskyLens.read()
            telemetry.addData("left", huskyLens.getColorPair().first)
            telemetry.addData("right", huskyLens.getColorPair().second)
            for (block in huskyLens.read()){
                telemetry.addData("blockx", block.x)
            }
            telemetry.update()
        }
    }
}