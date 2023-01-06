
console.log("start");

csv = "time ; pulses ; speed ; current ; target ; pwm ; position ; state \n";

/*
data["state"] = currentSystemState;
      data["pulses"] = currentPulses;
      data["posizione"] = getPercentagePosition() * 100;
      data["pwm"] = power;
      data["current"] = currentMeasured;
      data["target"] = radSecToRpm(targetSpeed);
      data["speed"] = radSecToRpm(currentSpeed);
      data["millis"] = millis();
*/



function addToCsv(json) {
    const obj = JSON.parse(json);
    console.log(json)

    var time = obj.millis;
    //var angle = obj.angle;
    var pulses = obj.pulses;
    var speed = parseFloat(obj.speed).toFixed(2);
    var current = parseFloat(obj.current).toFixed(2);
    var target = obj.target;
    var pwm = obj.pwm;
    var position = obj.posizione;
    var state = stateToString(obj.state);

    csv += time + ";" + pulses + ";" + speed + ";" + current + ";" + target + ";" + pwm + ";" + position + ";" + (state) + "\n";
}

function saveCsv() {
    var blob = new Blob([csv], { type: "text/plain;charset=utf-8" });
    saveAs(blob, "data.csv");
}

var saveCsv = (function () {
    var a = document.createElement("a");
    document.body.appendChild(a);
    a.style = "display: none";
    return function () {
        var blob = new Blob([csv], { type: "text/plain;charset=utf-8" }),
            url = window.URL.createObjectURL(blob);
        a.href = url;
        a.download = "data.csv";
        a.click();
        window.URL.revokeObjectURL(url);
    };
}());

function showData(json) {
    const obj = JSON.parse(json);

    addToCsv(json)
    console.log(json)

    $("#time").text(obj.millis);
    //$("#angle").text(parseFloat(obj.angle).format(2));
    $("#pulses").text(obj.pulses);
    var speedShow = parseFloat(obj.speed).toFixed(2);
    $("#speed").text(speedShow);
    var currentShow = parseFloat(obj.current).toFixed(2);
    $("#current").text(currentShow);
    var targetShow = parseFloat(obj.target).toFixed(2);
    $("#target").text(targetShow);
    var targetpwm = parseFloat(obj.pwm).toFixed(2);
    $("#pwm").text(targetpwm);
    var targetposition = parseFloat(obj.posizione).toFixed(2);
    $("#position").text(targetposition);

    var targettarget = parseFloat(obj.target).toFixed(2);
    $("#target").text(targettarget);
    //$("#control").text(obj.target == 0 ? "Torque" : "Velocity");

    var state = "";
    switch (obj.state) {
        case 0:
            state = "Start";
            break;
        case 1:
            state = "Inactive";
            break;
        case 2:
            state = "Inizio corsa";
            break;
        case 3:
            state = "Apertura";
            break;
        case 4:
            state = "Fine corsa";
            break;
        case 5:
            state = "Chiusura";
            break;
    }

    $("#state").text(state);
}

function stateToString(state) {
    switch (state) {
        case "Start":
            return "0";
        case "Inactive":
            return "1";
        case "Inizio corsa":
            return "2";
        case "Apertura":
            return "3";
        case "Fine corsa":
            return "4";
        case "Chiusura":
            return "5";
    }
}

function sendData() {

    //var vtocco = $("#input-vtocco").val();

    var inputTimeDuration = $("#input-timeDuration").val();
    var inputPulseStart = $("#input-pulseStart").val();
    var inputPulseEnd = $("#input-pulseEnd").val();
    var rpmOpen = $("#input-rpmOpen").val();
    var rpmClose = $("#input-rpmClose").val();
    var railStart = $("#input-railStart").val();
    var railEnd = $("#input-railEnd").val();


    //sscanf(command.c_str(), "Set;%d;%d;%d;%d;%d;%f;%f;", &tmptimeoutDuration, &tmppulseStart, &tmppulseEnd, &tmprpmOpen, &tmprpmClose, &tmprailStart, &tmprailEnd);

    var txt = "Set;" + inputTimeDuration + ";" + inputPulseStart + ";" + inputPulseEnd + ";" + rpmOpen + ";" + rpmClose + ";" + railStart + ";" + railEnd;

    $("#output-box").text(txt);

    console.log("sendData: " + txt);

    sendSetPacket(txt);
}

function setValues() {
    //parse txt string and set values to input fields
    var txt = $("#output-box").val();
    var values = txt.split(";");
    $("#input-timeDuration").val(values[1]);
    $("#input-pulseStart").val(values[2]);
    $("#input-pulseEnd").val(values[3]);
    $("#input-rpmOpen").val(values[4]);
    $("#input-rpmClose").val(values[5]);
    $("#input-railStart").val(values[6]);
    $("#input-railEnd").val(values[7]);
}

function clearlog() {
    $("#logDiv").text("");
}

function appendToLog(text) {
    $("#logDiv").append(text + "<br>");
}

eel.expose(showData);
eel.expose(appendToLog);

// funzioni collegamento a python

/*async function stopDrive() {
    await eel.stop_drive()();
}*/

/*async function enableDrive() {
    await eel.enable_drive()();
}*/

async function startDrive() {
    await eel.start_drive()();
}

async function resetDrive() {
    await eel.reset_drive()();
}

async function sendSetPacket(txt) {
    await eel.send_set_packet(txt)();
}

async function connectSerial() {
    await eel.connect_serial()();
}

async function disconnectSerial() {
    await eel.disconnect_serial()();
}