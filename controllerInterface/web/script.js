
console.log("start");
csvHeaderBortoluzzi = "time,pulses,speed,current,target,pwm1,pwm2,state,encoder,voltage\n";
csvHeaderSalice = "time,pulses,speed,voltage,target,control,state,enabled,phases\n";
csv = "";

var avgPacketTime = 10;
var lastPacketTime = 0;
var numPackets = 0;

var type = "UNKNOWN"

// add a new data packet to the csv file    
function addToCsv(json) {
    if (type == "UNKNOWN") {
        return;
    } else if (type == "BORTOLUZZI") {
        const obj = JSON.parse(json);

        var time = obj.millis;
        var pulses = obj.pulses;
        var speed = parseFloat(obj.speed).toFixed(2);
        var current = parseFloat(obj.current).toFixed(2);
        var target = obj.target;
        var pwm1 = obj.pwm1;
        var pwm2 = obj.pwm2;
        var state = stateToString(obj.state);
        var encoder = obj.encoder;
        var battery = obj.battery;

        csv += time + "," + pulses + "," + speed + "," + current + "," + target + "," + pwm1 + "," + pwm2 + "," + (state) + "," + (encoder) + "," + (battery) + "\n";
    } else if (type == "SALICE") {
        const obj = JSON.parse(json);

        var time = obj.millis;
        var pulses = obj.pulses;
        var speed = parseFloat(obj.speed).toFixed(2);
        var voltage = parseFloat(obj.voltage).toFixed(2);
        var target = parseFloat(obj.target).toFixed(2);
        var control = obj.target == 0 ? "Torque" : "Velocity";
        var state = stateToString(obj.state);
        var enabled = obj.enabled;
        var phases = obj.phases;

        csv += time + "," + pulses + "," + speed + "," + voltage + "," + target + "," + control + "," + state + "," + enabled + "," + phases + "\n";
    }
}

// pad a number with leading zeros
function pad(num, size) {
    num = num.toString();
    while (num.length < size) num = "0" + num;
    return num;
}

// save the csv file with the current date and time, uses a workaround for file download
var saveCsv = (function () {
    var date = new Date(Date.now());
    var datestring = pad(date.getFullYear(), 4) + "-" + pad(date.getMonth() + 1, 2) + "-" + pad(date.getDate(), 2) + "-" + pad(date.getHours(), 2) + "-" + pad(date.getMinutes(), 2) + "-" + pad(date.getSeconds(), 2);

    var filename = "CMB_motor_log_" + type + "_" + datestring + ".csv";

    var a = document.createElement("a");
    document.body.appendChild(a);
    a.style = "display: none";
    return function () {
        var blob = new Blob([csv], { type: "text/plain;charset=utf-8" }),
            url = window.URL.createObjectURL(blob);

        if (type == "BORTOLUZZI") {
            csv = csvHeaderBortoluzzi;
        } else if (type == "SALICE") {
            csv = csvHeaderSalice;
        } else {
            csv = "";
        }

        a.href = url;
        a.download = filename;
        a.click();
        window.URL.revokeObjectURL(url);
    };
}());

// refresh the list of available ports through python and display a radio button for each port
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

// display the data from the json packet in the html page
function showData(json) {
    const obj = JSON.parse(json);

    addToCsv(json)
    //console.log(json)

    //compute timing precision
    numPackets++;
    if (lastPacketTime != 0) {
        avgPacketTime = (avgPacketTime * (numPackets - 1) + obj.time - lastPacketTime) / numPackets;
    }
    lastPacketTime = obj.time;

    $("#time").text(obj.time);

    var state = "";
    var color = "";

    if (type == "BORTOLUZZI") {
        $("#pulses").text(obj.pulses);
        $("#speed").text(parseFloat(obj.speed).toFixed(2));
        $("#current").text(parseFloat(obj.current).toFixed(2));
        $("#target").text(parseFloat(obj.target).toFixed(2));
        $("#pwm1").text(parseFloat(obj.pwm1));
        $("#pwm2").text(parseFloat(obj.pwm2));
        $("#target").text(parseFloat(obj.target).toFixed(2));
        $("#encoder").text(parseFloat(obj.encoder).toFixed(2));
        $("#battery").text(parseInt(obj.battery));

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
            case 7:
                state = "Pausa";
                color = "brown";
                $("#start-button").show();
                $("#stop-button").hide();
                $("#startConfig-button").show();
                $("#endConfig-button").hide();
                $("#startContTest-button").hide();
                $("#endContTest-button").hide();
                break;
        }
    }
    else if (type == "SALICE") {
        $("#pulses").text(obj.pulses);
        $("#speed").text(parseFloat(obj.speed).toFixed(2));
        $("#voltage").text(parseFloat(obj.voltage).toFixed(2));
        $("#target").text(parseFloat(obj.target).toFixed(2));
        $("#control").text(obj.control == 0 ? "Torque" : "Velocity");

        switch (obj.state) {
            case 0:
                state = "Start";
                color = "white";
                break;
            case 1:
                state = "Inactive";
                color = "grey";
                break;
            case 2:
                state = "Spinta";
                color = "yellow";
                break;
            case 3:
                state = "Frenata";
                color = "greenyellow";
                break;
            case 4:
                state = "Quasi fine corsa";
                color = "red";
                break;
            case 5:
                state = "Fine corsa";
                color = "lightblue";
                break;
            case 6:
                state = "Inizio ritorno";
                color = "orange";
                break;
            case 7:
                state = "Ritorno Velocità";
                color = "brown";
                break;
            case 7:
                state = "Ritorno Torque";
                color = "blue";
                break;
        }
    }
    else {
        console.error("ERROR IN PARSING DATA");
        return;
    }

    $("#state").text(state);
    $("#state").parent().css("background-color", color);
}

// convert the state number to a string
function stateToString(state) {
    state = parseInt(state)

    if (type == "BORTOLUZZI") {
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
    } else if (type == "SALICE") {
        switch (state) {
            case 0:
                return "Start";
            case 1:
                return "Inattivo";
            case 2:
                return "Spinta";
            case 3:
                return "Frenata";
            case 4:
                return "Quasi fine corsa";
            case 5:
                return "Fine corsa";
            case 6:
                return "Inizio ritorno";
            case 7:
                return "Ritorno velocità";
            case 8:
                return "Ritorno torque";
        }
    }

    return "UNKNOWN";
}

// send the data from the input fields to the python script
function sendData() {
    if (type == "BORTOLUZZI") {
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
    }
    else if (type == "SALICE") {
        var railLength = $("#input-rail").val();

        var vmax = $("#input-vmax").val();
        var vmin = $("#input-vmin").val();
        var pulseStart = railLength * $("#input-pulseStart").val();
        var pulseStop = railLength * $("#input-pulseStop").val();
        var pulseEnd = railLength * $("#input-pulseEnd").val();
        var tend = $("#input-tend").val();
        var tbrake = $("#input-tbrake").val();
        var timeoutDuration = $("#input-timeoutDuration").val();
        var vmaxfrenata = $("#input-vmaxfrenata").val();
        var vminfrenata = $("#input-vminfrenata").val();
        var cfrenata = $("#input-cfrenata").val();
        var vtocco = $("#input-vtocco").val();

        //sscanf(command.c_str(), "Set;%f;%f;%f;%d;%d;%d;%f;%f;%d;%f;%f;%f;%f", &tmpvmax, &tmpvmin, &tmprampDuration, &tmppulseStart, &tmppulseStop, &tmppulseEnd, &tmptend, &tmptbrake, &tmptimeoutDuration, &tmpvmaxfrenata, &tmpvminfrenata, &tmpcfrenata, &tmpvtocco);

        var txt = "Set;" + vmax + ";" + vmin + ";" + 0 + ";" + pulseStart + ";" + pulseStop + ";" + pulseEnd + ";" + tend + ";" + tbrake + ";" + timeoutDuration + ";" + vmaxfrenata + ";" + vminfrenata + ";" + cfrenata + ";" + vtocco;
    } else {
        console.error("ERROR IN SENDING DATA");
        return;
    }


    $("#output-box").val(txt);

    console.log("sendData: " + txt);

    sendSetPacket(txt);
}

// parse txt string and set values to input fields
function setValues() {
    if (type == "BORTOLUZZI") {
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
    } else if (type == "SALICE") {
        var railLength = $("#input-rail").val();
        var txt = $("#output-box").val();
        var values = txt.split(";");
        $("#input-vmax").val(values[1]);
        $("#input-vmin").val(values[2]);
        $("#input-ramp").val(values[3]);
        $("#input-pulseStart").val((values[4] / railLength).toFixed(2));
        $("#input-pulseStop").val((values[5] / railLength).toFixed(2));
        $("#input-pulseEnd").val((values[6] / railLength).toFixed(2));
        $("#input-tend").val(values[7]);
        $("#input-tbrake").val(values[8]);
        $("#input-timeoutDuration").val(values[9]);
        $("#input-vmaxfrenata").val(values[10]);
        $("#input-vminfrenata").val(values[11]);
        $("#input-cfrenata").val(values[12]);
        $("#input-vtocco").val(values[13]);
    }
    else {
        console.error("ERROR IN SETTING VALUES");
        return;
    }
}

// clear the log div
function clearlog() {
    $("#logDiv").text("");
}

// parse the parameters from a Get; packet and set the values to the input fields
function parseGet(text) {
    if (type == "BORTOLUZZI") {
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
    } else if (type == "SALICE") {
        var railLength = $("#input-rail").val();
        var values = text.split(";");
        $("#input-vmax").val(values[1]);
        $("#input-vmin").val(values[2]);
        $("#input-ramp").val(values[3]);
        $("#input-pulseStart").val((values[4] / railLength).toFixed(2));
        $("#input-pulseStop").val((values[5] / railLength).toFixed(2));
        $("#input-pulseEnd").val((values[6] / railLength).toFixed(2));
        $("#input-tend").val(values[7]);
        $("#input-tbrake").val(values[8]);
        $("#input-timeoutDuration").val(values[9]);
        $("#input-vmaxfrenata").val(values[10]);
        $("#input-vminfrenata").val(values[11]);
        $("#input-cfrenata").val(values[12]);
        $("#input-vtocco").val(values[13]);
    } else {
        console.error("ERROR IN PARSING GET");
        return;
    }

}

function appendToLog(text) {

    //if text starts with Get; parse the parameters
    if (text.indexOf("Get;") != -1) {
        parseGet(text);
    }


    // place text in div with time (hh:mm:ss)
    var appendElement = "<span class='log-time'>" + new Date().toLocaleTimeString() + "</span> " + text + "<br>";

    $("#logDiv").append(appendElement);
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

function setType(newType) {
    if (type == newType) {
        return;
    }

    switch (newType) {
        case "BORTOLUZZI":
            type = "BORTOLUZZI"
            $(".typeSalice").hide();
            $(".typeBortoluzzi").show();
            csv = csvHeaderBortoluzzi;
            break;
        case "SALICE":
            type = "SALICE"
            $(".typeSalice").show();
            $(".typeBortoluzzi").hide();
            csv = csvHeaderSalice;
            break;
        default:
            type = "UNKNOWN"
            $(".typeSalice").hide();
            $(".typeBortoluzzi").hide();
            appendToLog("Device type not recognized")
            break;
    }
}

$(function () {
    refreshPorts();
})

// funzioni collegamento a python

eel.expose(showData);
eel.expose(appendToLog);
eel.expose(showConnected);
eel.expose(showDisconnected);
eel.expose(setType);

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