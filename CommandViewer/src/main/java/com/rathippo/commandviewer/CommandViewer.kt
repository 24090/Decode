package com.rathippo.commandviewer

import android.content.Context
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications
import com.rathippo.commandviewer.CommandViewer.commandLog
import fi.iki.elonen.NanoHTTPD.newFixedLengthResponse
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.addJsonObject
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put
import kotlinx.serialization.json.putJsonArray
import org.firstinspires.ftc.ftccommon.external.OnCreate
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop


open class CommandViewerParams{
    companion object {
        var customCss = ""
    }
}

object CommandViewer : Notifications {
    val commandLog = ArrayList<CommandMessage>()

    private var server: Server? = null
    val active: Boolean = server?.sockets?.isEmpty() ?: false
    @OnCreate
    @JvmStatic fun start(context: Context?) {
        val getFile = { path: String -> context!!.assets.open(path).bufferedReader().readText()}
        val page = newFixedLengthResponse(
            getFile("main.html")
            .replace(
                "@INSERT_JSONPATCH",
                getFile("jsonpatch.min.js")
            )
            .replace(
                "@INSERT_JS",
                getFile("main.js")
            )
            .replace(
                "@INSERT_CSS",
                getFile("style.css")
            )
            .replace(
                "@INSERT_CUSTOMCSS",
                CommandViewerParams.customCss
            )
        )
        if (server == null && context != null) {
            server = Server(page)
        }
    }

    @OnCreateEventLoop
    fun onEventLoopRegistered(ftcEventLoop: FtcEventLoop){
        ftcEventLoop.opModeManager.registerListener(this)
    }


    private val functions = ArrayList<() -> Unit>()
    fun runFunction(i: Int){
        // intentional: allows no function to be encoded as int
        if (i < 0) {
            return
        }
        functions[i].invoke()
    }
    fun registerFunction(function: () -> Unit): Int{
        functions.add(function)
        return functions.size - 1
    }
    fun sanitizeString(s: String): String{
        return s.replace("\\","\\\\")
    }
    fun update(){
        if (!active) return

        val msg: JsonElement = buildJsonObject {
            putJsonArray("messages"){
                for (cm in commandLog) { addJsonObject {
                    when (cm) {
                        is CommandMessage.Action -> {
                            put("type", "Action")
                            put("message", sanitizeString(cm.message))
                            put("i", cm.function)
                        }
                        is CommandMessage.Begin -> {
                            put("type", "Begin")
                            put("uid", cm.uid)
                            put("name", cm.name)
                            put("class", cm.cssClass)
                            put("selfCondense", cm.selfCondense)
                            putJsonArray("buttons") {
                                for (button in cm.buttons) {
                                    addJsonObject {
                                        put("i", button.first)
                                        put("message", button.second)
                                    }
                                }
                            }
                        }

                        CommandMessage.End -> {
                            put("type", "End")
                        }

                        is CommandMessage.Error -> {
                            put("type", "Error")
                            put("message", sanitizeString(cm.error.toString()))
                        }

                        is CommandMessage.Log -> {
                            put("type", "Log")
                            put("message", sanitizeString(cm.message))
                        }
                    }
                }}
            }
        }
        server?.update(msg)
        commandLog.clear()
    }

    override fun onOpModePreInit(opMode: OpMode?) {}
    override fun onOpModePreStart(opMode: OpMode?) {}
    override fun onOpModePostStop(opMode: OpMode?) {
        commandLog.clear()
        functions.clear()
    }
}

sealed class CommandMessage{
    fun send(){
        commandLog.add(this)
    }

    /**
     * Info sent to CommandViewer at the beginning of a command
     * @param name the name of the command e.g. SlideUp
     * @param selfCondense whether or not to display commands with the same name as if they were one (for example a sequence or parallel command might use this)
     * @param buttons the buttons attached to this command (e.g. pause, skip, resume)
     * @param cssClass any css class the command should have default options are "" and dead
     * @param uid a unique number used to identify this command across updates
     * */
    class Begin(
        val uid: Int,
        val name: String,
        val buttons: List<Pair<Int, String>>,
        val selfCondense: Boolean = false,
        val cssClass: String = "",
    ): CommandMessage()
    object End : CommandMessage()
    class Error(val error: Throwable): CommandMessage()
    class Log(val message: String): CommandMessage()
    class Action(val message: String, val function: Int): CommandMessage()
}