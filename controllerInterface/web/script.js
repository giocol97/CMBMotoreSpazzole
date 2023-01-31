
console.log("start");
csvHeader = "time,pulses,speed,current,target,pwm1,pwm2,state,encoder,voltage\n";
csv = csvHeader;

var avgPacketTime = 10;
var lastPacketTime = 0;
var numPackets = 0;

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
    //console.log(json)

    var time = obj.millis;
    //var angle = obj.angle;
    var pulses = obj.pulses;
    var speed = parseFloat(obj.speed).toFixed(2);
    var current = parseFloat(obj.current).toFixed(2);
    var target = obj.target;
    //var pwm = obj.pwm;
    var pwm1 = obj.pwm1;
    var pwm2 = obj.pwm2;
    //var position = obj.posizione;
    var state = stateToString(obj.state);
    var encoder = obj.encoder;
    var battery = obj.battery;

    csv += time + "," + pulses + "," + speed + "," + current + "," + target + "," + pwm1 + "," + pwm2 + "," /*+ position + "," */+ (state) + "," + (encoder) + "," + (battery) + "\n";
}

/*function saveCsv() {
    var blob = new Blob([csv], { type: "text/plain;charset=utf-8" });
    saveAs(blob, "data.csv");
}*/

function pad(num, size) {
    num = num.toString();
    while (num.length < size) num = "0" + num;
    return num;
}

var saveCsv = (function () {
    var date = new Date(Date.now());
    var datestring = pad(date.getFullYear(), 4) + "-" + pad(date.getMonth() + 1, 2) + "-" + pad(date.getDate(), 2) + "-" + pad(date.getHours(), 2) + "-" + pad(date.getMinutes(), 2) + "-" + pad(date.getSeconds(), 2);

    var filename = "CMB_motor_log_" + datestring + ".csv";

    var a = document.createElement("a");
    document.body.appendChild(a);
    a.style = "display: none";
    return function () {
        var blob = new Blob([csv], { type: "text/plain;charset=utf-8" }),
            url = window.URL.createObjectURL(blob);

        csv = csvHeader;
        a.href = url;
        a.download = filename;
        a.click();
        window.URL.revokeObjectURL(url);
    };


}());

async function refreshPorts() {
    let portsString = await getAvailablePorts();

    $("#com-port").text("")
    //parse ports from ["COM3", "COM16"] to list
    var ports = portsString.substring(1, portsString.length - 1).split(",");

    for (var i = 0; i < ports.length; i++) {
        var port = ports[i].trim()
        port = port.substring(1, port.length - 1)
        $("#com-port").append("<input style='margin-left:0.3em;margin-right:0.3em;' type='radio' id='com-" + port + "' name='com-port' value='" + port + "'><label for='com-" + port + "'>" + port + "</label>");
    }

    $("#com-port input:radio:first").prop("checked", true);
}

function showData(json) {
    const obj = JSON.parse(json);

    addToCsv(json)
    //console.log(json)

    //compute timing precision
    numPackets++;
    if (lastPacketTime != 0) {
        avgPacketTime = (avgPacketTime * (numPackets - 1) + obj.millis - lastPacketTime) / numPackets;
    }
    lastPacketTime = obj.millis;

    $("#time").text(obj.millis);
    //$("#angle").text(parseFloat(obj.angle).format(2));
    $("#pulses").text(obj.pulses);
    var speedShow = parseFloat(obj.speed).toFixed(2);
    $("#speed").text(speedShow);
    var currentShow = parseFloat(obj.current).toFixed(2);
    $("#current").text(currentShow);
    var targetShow = parseFloat(obj.target).toFixed(2);
    $("#target").text(targetShow);
    //var targetpwm = parseFloat(obj.pwm).toFixed(2);
    //$("#pwm").text(targetpwm);
    var targetpwm1 = parseFloat(obj.pwm1);
    $("#pwm1").text(targetpwm1);
    var targetpwm2 = parseFloat(obj.pwm2);
    $("#pwm2").text(targetpwm2);
    //var targetposition = parseFloat(obj.posizione).toFixed(2);
    //$("#position").text(targetposition);

    var targettarget = parseFloat(obj.target).toFixed(2);
    $("#target").text(targettarget);

    var encoderPosition = parseFloat(obj.encoder).toFixed(2);
    $("#encoder").text(encoderPosition);

    var batteryVoltage = parseInt(obj.battery);
    $("#battery").text(batteryVoltage);

    //$("#control").text(obj.target == 0 ? "Torque" : "Velocity");

    var state = "";
    var color = "";
    switch (obj.state) {
        case 0:
            state = "Start";
            color = "white";
            break;
        case 1:
            state = "Inactive";
            color = "grey";
            $("#start-button").show();
            $("#stop-button").hide();
            $("#startConfig-button").show();
            $("#endConfig-button").hide();
            $("#startContTest-button").hide();
            $("#endContTest-button").hide();
            break;
        case 2:
            state = "Inizio corsa";
            color = "yellow";
            $("#start-button").hide();
            $("#startConfig-button").hide();
            $("#stop-button").show();
            $("#startContTest-button").show();
            break;
        case 3:
            state = "Apertura";
            color = "greenyellow";
            $("#start-button").hide();
            $("#startConfig-button").hide();
            $("#stop-button").show();
            $("#startContTest-button").show();
            break;
        case 4:
            state = "Fine corsa";
            color = "red";
            $("#start-button").hide();
            $("#startConfig-button").hide();
            $("#stop-button").show();
            $("#startContTest-button").show();
            break;
        case 5:
            state = "Chiusura";
            color = "lightblue";
            $("#start-button").hide();
            $("#startConfig-button").hide();
            $("#stop-button").show();
            $("#startContTest-button").show();
            break;
        case 6:
            state = "Configurazione";
            color = "orange";
            $("#start-button").hide();
            $("#startConfig-button").hide();
            $("#endConfig-button").show();
            $("#startContTest-button").hide();
            break;
    }

    $("#state").text(state);
    $("#state").parent().css("background-color", color);
}

function stateToString(state) {
    state = parseInt(state)
    switch (state) {
        case 0:
            return "Start";
        case 1:
            return "Inattivo";
        case 2:
            return "Inizio corsa";
        case 3:
            return "Apertura";
        case 4:
            return "Fine corsa";
        case 5:
            return "Chiusura";
        case 6:
            return "Configurazione";
    }
}

function sendData() {

    //var vtocco = $("#input-vtocco").val();

    //variabili controllo algoritmo
    var inputTimeDuration = $("#input-timeDuration").val();
    var inputTimeOpen = $("#input-timeOpen").val();
    var inputPulseStart = $("#input-pulseStart").val();
    var inputPulseEnd = $("#input-pulseEnd").val();
    var rpmOpen = $("#input-rpmOpen").val();
    var rpmClose = $("#input-rpmClose").val();
    var railStart = $("#input-railStart").val();
    var railEnd = $("#input-railEnd").val();

    //pid open
    var tmppidOpenKp = $("#input-pidOpenKp").val();
    var tmppidOpenKi = $("#input-pidOpenKi").val();
    var tmppidOpenKd = $("#input-pidOpenKd").val();

    //pid close
    var tmppidCloseKp = $("#input-pidCloseKp").val();
    var tmppidCloseKi = $("#input-pidCloseKi").val();
    var tmppidCloseKd = $("#input-pidCloseKd").val();


    //sscanf(command.c_str(), "Set;%d;%d;%d;%d;%d;%d;%f;%f;%f;%f;%f;%f;%f;%f;", &tmptimeoutDuration, &tmptimeoutOpen, &tmppulseStart, &tmppulseEnd, &tmprpmOpen, &tmprpmClose, &tmprailStart, &tmprailEnd, &tmppidOpenKp, &tmppidOpenKi, &tmppidOpenKd, &tmppidCloseKp, &tmppidCloseKi, &tmppidCloseKd);

    var txt = "Set;" + inputTimeDuration + ";" + inputTimeOpen + ";" + inputPulseStart + ";" + inputPulseEnd + ";" + rpmOpen + ";" + rpmClose + ";" + railStart + ";" + railEnd + ";" + tmppidOpenKp + ";" + tmppidOpenKi + ";" + tmppidOpenKd + ";" + tmppidCloseKp + ";" + tmppidCloseKi + ";" + tmppidCloseKd + ";";

    $("#output-box").val(txt);

    console.log("sendData: " + txt);

    sendSetPacket(txt);
}

function setValues() {
    //parse txt string and set values to input fields
    var txt = $("#output-box").val();
    var values = txt.split(";");
    $("#input-timeDuration").val(values[1]);
    $("#input-timeOpen").val(values[2]);
    $("#input-pulseStart").val(values[3]);
    $("#input-pulseEnd").val(values[4]);
    $("#input-rpmOpen").val(values[5]);
    $("#input-rpmClose").val(values[6]);
    $("#input-railStart").val(values[7]);
    $("#input-railEnd").val(values[8]);
    $("#input-pidOpenKp").val(values[9]);
    $("#input-pidOpenKi").val(values[10]);
    $("#input-pidOpenKd").val(values[11]);
    $("#input-pidCloseKp").val(values[12]);
    $("#input-pidCloseKi").val(values[13]);
    $("#input-pidCloseKd").val(values[14]);
}

function clearlog() {
    $("#logDiv").text("");
}

function parseGet(text) {
    var values = text.split(";");
    $("#input-timeDuration").val(values[1]);
    $("#input-timeOpen").val(values[2]);
    $("#input-pulseStart").val(values[3]);
    $("#input-pulseEnd").val(values[4]);
    $("#input-rpmOpen").val(values[5]);
    $("#input-rpmClose").val(values[6]);
    $("#input-railStart").val(values[7]);
    $("#input-railEnd").val(values[8]);
    $("#input-pidOpenKp").val(values[9]);
    $("#input-pidOpenKi").val(values[10]);
    $("#input-pidOpenKd").val(values[11]);
    $("#input-pidCloseKp").val(values[12]);
    $("#input-pidCloseKi").val(values[13]);
    $("#input-pidCloseKd").val(values[14]);
}

function appendToLog(text) {

    //if text starts with Get; parse the parameters
    if (text.indexOf("Get;") != -1) {
        parseGet(text);
    }

    $("#logDiv").append(text + "<br>");
}

function showConnected(port) {
    $("#com-port").text("Connected to " + port);
    $("#connect-button").hide();
    $("#disconnect-button").show();
}

function showDisconnected() {
    refreshPorts();
    $("#connect-button").show();
    $("#disconnect-button").hide();
}

eel.expose(showData);
eel.expose(appendToLog);
eel.expose(showConnected);
eel.expose(showDisconnected);

$(function () {
    refreshPorts();
})

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

async function stopDrive() {
    await eel.stop_drive()();
}

async function startConfig() {
    await eel.start_config()();
}

async function endConfig() {
    await eel.end_config()();
}

async function startContTest() {
    $("#startContTest-button").hide();
    $("#endContTest-button").show();
    await eel.start_contTest()();
}

async function endContTest() {
    $("#endContTest-button").hide();
    await eel.end_contTest()();
}


async function resetDrive() {
    await eel.reset_drive()();
}

async function sendSetPacket(txt) {
    await eel.send_set_packet(txt)();
}

async function getData() {
    await eel.get_data()();
}

async function connectSerial() {
    var port = $('input[name="com-port"]:checked').val();
    await eel.connect_serial(port)();
}

async function disconnectSerial() {
    await eel.disconnect_serial()();
}

async function getAvailablePorts() {
    return await eel.get_available_ports()();
}