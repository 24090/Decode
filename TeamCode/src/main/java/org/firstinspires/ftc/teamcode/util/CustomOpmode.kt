package org.firstinspires.ftc.teamcode.util

import android.graphics.Path
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.reflect.KClass

// 1: initialize
// 2: runopmode

//open class CustomOpMode {
//    fun init() {
//        TODO("Not yet implemented")
//    }
//
//    fun loop() {
//        TODO("Not yet implemented")
//    }
//}
//
//fun TeleOp(val customOpModeClass: KClass<CustomOpMode>) = @TeleOp  object : LinearOpMode() {
//    override fun runOpMode() {
//        val instance = customOpModeClass.constructors.first().call()
//        waitForStart()
//    }
//
//}