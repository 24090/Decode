var json = {"messages":[]}
var collapseInfo = {}
// Create WebSocket connection.
const socket = new WebSocket("/");

function reset(){
    json = {"messages":[]}
    collapseInfo = {}
    socket = new WebSocket("/");
}

function toggleCollapse(targetUID){
  let target = document.getElementById(`collapsediv-${targetUID}`)
  if (target.classList.contains("collapsed")){
    target.classList.remove("collapsed")
    collapseInfo[targetUID] = false
  } else {
    target.classList.add("collapsed")
    collapseInfo[targetUID] = true
  }
}

function runFunction(index){
  socket.send(index)
}

function renderMessage(message, rootElement, nestingLevel){
  switch (message.type) {
    case "Begin":
      let div = rootElement.appendChild(document.createElement("div"))
      div.id = message.uid
      div.className = message.class
      div.classList.add("command")
      let dead = div.classList.contains("dead")
      div.style = `--nesting-level: ${nestingLevel};`
      let title = document.createElement("h3")
      title.innerText = message.name
      div.appendChild(title)
      
      let collapsebutton = div.appendChild(document.createElement("button"))
      collapsebutton.innerText = "v"
      collapsebutton.setAttribute("onclick", `toggleCollapse(${message.uid})`)
      
      for (button_index in message.buttons) {
        let button_info = message.buttons[button_index]
        let button = div.appendChild(document.createElement("button"))
        button.setAttribute("onclick", `runFunction(${button_info.i})`)
        button.innerText = button_info.message
      }
      
      collapsediv = div.appendChild(document.createElement("div"))
      if (collapseInfo.hasOwnProperty(message.uid)){
        collapsediv.className = collapseInfo[message.uid] ? "collapsediv collapsed": "collapsediv"
        if (collapseInfo[message.uid] == dead){
          delete collapseInfo[message.uid]
        }
      } else {
        collapsediv.className = dead ? "collapsediv collapsed": "collapsediv"
      }
      collapsediv.id = `collapsediv-${message.uid}`
      rootElement = collapsediv
      nestingLevel += 1
      break;
    case "End":
      nestingLevel -= 1
      rootElement = rootElement.parentElement.parentElement
      break;
    case "Error":
      let error = rootElement.appendChild(document.createElement("p"))
      error.className = "error"
      error.innerText = message.message
      break;
    case "Log":
      let log = rootElement.appendChild(document.createElement("p"))
      log.className = "log"
      log.innerText = message.message
      break;
    case "Action":
      let button = rootElement.appendChild(document.createElement("button"))
      button.className = "action"
      button.setAttribute("onclick", `runFunction(${message.i})`)
      button.innerText = message.message
      break;
  }
  return {rootElement, nestingLevel}
}

// Connection opened
socket.onopen = (event) => console.log("Opened", event);
// Listen for messages
socket.onmessage = (event) => {
    let ops = JSON.parse(event.data)
    json = jsonpatch.apply_patch(json, ops);
    let first_change = Math.min( ... ops.map((v) => {
      switch (v.op){
        case "move":
          return min(v.from.split("/")[2], v.path.split("/")[2])
        case "update":
          return Number.MAX_SAFE_INTEGER
        default:
          return v.path.split("/")[2]
      }
    }))
    let nestingLevel = 0;
    let rootElement = document.createElement("section")
    rootElement.id = "commands"
    rootElement.innerHTML = ""
    for (const index in json.messages) {
      ({rootElement, nestingLevel} = renderMessage(json.messages[index], rootElement, nestingLevel));
    }
    document.getElementById("commands").innerHTML = rootElement.innerHTML
}

socket.onclose = (event) => console.log("Closed", event)