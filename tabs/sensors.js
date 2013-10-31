function tab_initialize_sensors() {
    ga_tracker.sendAppView('Sensor Page');
    
    // Setup variables
    samples_gyro_i = 300;
    samples_accel_i = 300;
    samples_mag_i = 300;
    samples_baro_i = 300;
    samples_debug_i = 300;
    
    gyro_data = new Array(3);
    accel_data = new Array(3);
    mag_data = new Array(3);
    baro_data = new Array(1);
    debug_data = new Array(4);
    
    gyro_data[0] = new Array();
    gyro_data[1] = new Array();
    gyro_data[2] = new Array();
    
    accel_data[0] = new Array();
    accel_data[1] = new Array();
    accel_data[2] = new Array();  
    
    mag_data[0] = new Array();
    mag_data[1] = new Array();
    mag_data[2] = new Array();    

    baro_data[0] = new Array();
    for (var i = 0; i < 4; i++) debug_data[i] = new Array();
    
    for (var i = 0; i <= 300; i++) {
        gyro_data[0].push([i, 0]);
        gyro_data[1].push([i, 0]);
        gyro_data[2].push([i, 0]);
        
        accel_data[0].push([i, 0]);
        accel_data[1].push([i, 0]);
        accel_data[2].push([i, 0]); 

        mag_data[0].push([i, 0]);
        mag_data[1].push([i, 0]);
        mag_data[2].push([i, 0]);   

        baro_data[0].push([i, 0]);
        for (var j = 0; j < 4; j++) debug_data[j].push([i, 0]);
    }
    
    // plot specific stuff
    e_graph_gyro = document.getElementById("gyro");
    e_graph_accel = document.getElementById("accel");  
    e_graph_mag = document.getElementById("mag");
    e_graph_baro = document.getElementById("baro");  
    e_graph_debug1 = document.getElementById("debug1");
    e_graph_debug2 = document.getElementById("debug2");
    e_graph_debug3 = document.getElementById("debug3");
    e_graph_debug4 = document.getElementById("debug4");
    
    gyro_options = {
        title: "Gyroscope (deg/s)",
        shadowSize: 0,
        yaxis : {
            tickDecimals: 0
        },
        xaxis : {
            //noTicks = 0
        },
        grid : {
            backgroundColor: "#FFFFFF"
        },
        legend : {
            position: "we",
            backgroundOpacity: 0
        }
    }
    
    accel_options = {
        title: "Accelerometer (g)",
        shadowSize: 0,
        yaxis : {
            tickDecimals: 1,
            max : 1.5,
            min : -1.5
        },
        xaxis : {
            //noTicks = 0
        },
        grid : {
            backgroundColor : "#FFFFFF"
        },
        legend : {
            position: "we",
            backgroundOpacity: 0
        }
    } 
    
    mag_options = {
        title: "Magnetometer (Ga)",
        shadowSize: 0,
        yaxis : {
            tickDecimals: 0
        },
        xaxis : {
            //noTicks = 0
        },
        grid : {
            backgroundColor : "#FFFFFF"
        },
        legend : {
            position: "we",
            backgroundOpacity: 0
        }
    }
    
    baro_options = {
        title: "Barometer (meters)",
        shadowSize: 0,
        yaxis : {
            tickDecimals: 1,
        },
        xaxis : {
            //noTicks = 0
        },
        grid : {
            backgroundColor : "#FFFFFF"
        },
        legend : {
            position: "we",
            backgroundOpacity: 0
        }
    }     

    debug1_options = {
        title: "Debug1",
        shadowSize: 0,
        yaxis : {
            tickDecimals: 1,
        },
        xaxis : {
            //noTicks = 0
        },
        grid : {
            backgroundColor : "#FFFFFF"
        },
        legend : {
            position: "we",
            backgroundOpacity: 0
        }
    }
    debug2_options = {};
    for (var key in debug1_options) debug2_options[key] = debug1_options[key];
    debug2_options.title = "Debug2";

    debug3_options = {};
    for (var key in debug1_options) debug3_options[key] = debug1_options[key];
    debug3_options.title = "Debug3";

    debug4_options = {};
    for (var key in debug1_options) debug4_options[key] = debug1_options[key];
    debug4_options.title = "Debug4";
    
    // set refresh speeds according to configuration saved in storage
    chrome.storage.local.get('sensor_refresh_rates', function(result) {
        if (typeof result.sensor_refresh_rates != 'undefined') {
            $('.tab-sensors select').eq(0).val(result.sensor_refresh_rates.gyro); // gyro 
            $('.tab-sensors select').eq(1).val(result.sensor_refresh_rates.accel); // accel
            $('.tab-sensors select').eq(2).val(result.sensor_refresh_rates.mag); // mag
            $('.tab-sensors select').eq(3).val(result.sensor_refresh_rates.baro); // baro
            $('.tab-sensors select').eq(4).val(result.sensor_refresh_rates.debug); // debug
            
            $('.tab-sensors select').change(); // start polling data by triggering refresh rate change event
        } else {
            // start polling immediatly (as there is no configuration saved in the storage)
            $('.tab-sensors select').change(); // start polling data by triggering refresh rate change event
        }
    });
    
    $('.tab-sensors select').change(function() {
        // if any of the select fields change value, all of the select values are grabbed
        // and timers are re-initialized with the new settings
        
        var rates = {
            'gyro':  parseInt($('.tab-sensors select').eq(0).val()), 
            'accel': parseInt($('.tab-sensors select').eq(1).val()), 
            'mag':   parseInt($('.tab-sensors select').eq(2).val()), 
            'baro':  parseInt($('.tab-sensors select').eq(3).val()),
            'debug':  parseInt($('.tab-sensors select').eq(4).val())
        };
        
        
        // handling of "data pulling" is a little bit funky here, as MSP_RAW_IMU contains values for gyro/accel/mag but not baro
        // this means that setting a slower refresh rate on any of the attributes would have no effect
        // what we will do instead is = determinate the fastest refresh rate for those 3 attributes, use that as a "polling rate"
        // and use the "slower" refresh rates only for re-drawing the graphs (to save resources/computing power)
        var fastest = rates.gyro;
        
        if (rates.accel < fastest) {
            fastest = rates.accel;
        }
        
        if (rates.mag < fastest) {
            fastest = rates.mag;
        }
        
        // timer initialization
        disable_timers();
        
        // data pulling timers
        timers.push(setInterval(sensor_status_pull, 50));
        timers.push(setInterval(sensor_IMU_pull, fastest));
        timers.push(setInterval(sensor_altitude_pull, rates.baro));
        timers.push(setInterval(sensor_debug_pull, rates.debug));
        
        // processing timers
        timers.push(setInterval(sensor_process_gyro, rates.gyro));
        timers.push(setInterval(sensor_process_accel, rates.accel));
        timers.push(setInterval(sensor_process_mag, rates.mag));
        
        // store current/latest refresh rates in the storage
        chrome.storage.local.set({'sensor_refresh_rates': rates}, function() {
        });
    });
}

function sensor_status_pull() {
    send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
}

function sensor_IMU_pull() {
    send_message(MSP_codes.MSP_RAW_IMU, MSP_codes.MSP_RAW_IMU);
}

function sensor_altitude_pull() {
    send_message(MSP_codes.MSP_ALTITUDE, MSP_codes.MSP_ALTITUDE);
    
    // we can process this one right here
    sensor_process_baro();
}

function sensor_debug_pull() {
    send_message(MSP_codes.MSP_DEBUG, MSP_codes.MSP_DEBUG);
    
    // we can process this one right here
    sensor_process_debug();
}


function sensor_process_gyro() {
    gyro_data[0].push([samples_gyro_i, SENSOR_DATA.gyroscope[0]]);
    gyro_data[1].push([samples_gyro_i, SENSOR_DATA.gyroscope[1]]);
    gyro_data[2].push([samples_gyro_i, SENSOR_DATA.gyroscope[2]]);
    
    // Remove old data from array
    while (gyro_data[0].length > 300) {
        gyro_data[0].shift();
        gyro_data[1].shift();
        gyro_data[2].shift();
    } 

    Flotr.draw(e_graph_gyro, [ 
        {data: gyro_data[0], label: "X - rate [" + SENSOR_DATA.gyroscope[0].toFixed(2) + "]"}, 
        {data: gyro_data[1], label: "Y - rate [" + SENSOR_DATA.gyroscope[1].toFixed(2) + "]"}, 
        {data: gyro_data[2], label: "Z - rate [" + SENSOR_DATA.gyroscope[2].toFixed(2) + "]"} ], gyro_options); 
    
    samples_gyro_i++;
}

function sensor_process_accel() {
    accel_data[0].push([samples_accel_i, SENSOR_DATA.accelerometer[0]]);
    accel_data[1].push([samples_accel_i, SENSOR_DATA.accelerometer[1]]);
    accel_data[2].push([samples_accel_i, SENSOR_DATA.accelerometer[2]]);
    
    // Remove old data from array
    while (accel_data[0].length > 300) {        
        accel_data[0].shift();
        accel_data[1].shift();
        accel_data[2].shift(); 
    } 

    Flotr.draw(e_graph_accel, [ 
        {data: accel_data[1], label: "X - acceleration [" + SENSOR_DATA.accelerometer[0].toFixed(2) + "]"}, 
        {data: accel_data[0], label: "Y - acceleration [" + SENSOR_DATA.accelerometer[1].toFixed(2) + "]"}, 
        {data: accel_data[2], label: "Z - acceleration [" + SENSOR_DATA.accelerometer[2].toFixed(2) + "]"} ], accel_options);

    samples_accel_i++;
}

function sensor_process_mag() {
    mag_data[0].push([samples_mag_i, SENSOR_DATA.magnetometer[0]]);
    mag_data[1].push([samples_mag_i, SENSOR_DATA.magnetometer[1]]);
    mag_data[2].push([samples_mag_i, SENSOR_DATA.magnetometer[2]]);
    
    // Remove old data from array
    while (mag_data[0].length > 300) {
        mag_data[0].shift();
        mag_data[1].shift();
        mag_data[2].shift();
    }  

    Flotr.draw(e_graph_mag, [ 
        {data: mag_data[1], label: "X - Ga [" + SENSOR_DATA.magnetometer[0].toFixed(2) + "]"}, 
        {data: mag_data[0], label: "Y - Ga [" + SENSOR_DATA.magnetometer[1].toFixed(2) + "]"}, 
        {data: mag_data[2], label: "Z - Ga [" + SENSOR_DATA.magnetometer[2].toFixed(2) + "]"} ], mag_options); 

    samples_mag_i++;
}

function sensor_process_baro() {
    baro_data[0].push([samples_baro_i, SENSOR_DATA.altitude]);
    
    // Remove old data from array
    while (baro_data[0].length > 300) {
        baro_data[0].shift();
    } 

    Flotr.draw(e_graph_baro, [ 
        {data: baro_data[0], label: "X - meters [" + SENSOR_DATA.altitude.toFixed(2) + "]"} ], baro_options);

    samples_baro_i++;
}

function sensor_process_debug() {
    for (var i = 0; i < 4; i++) {
      debug_data[i].push([samples_debug_i, SENSOR_DATA.debug[i]]);

      // Remove old data from array
      while (debug_data[i].length > 300) {
          debug_data[i].shift();
      }
    }


    Flotr.draw(e_graph_debug1, [
        {data: debug_data[0], label: "debug1 [" + SENSOR_DATA.debug[0] + "]"} ], debug1_options);
    Flotr.draw(e_graph_debug2, [
        {data: debug_data[1], label: "debug2 [" + SENSOR_DATA.debug[1] + "]"} ], debug2_options);
    Flotr.draw(e_graph_debug3, [
        {data: debug_data[2], label: "debug3 [" + SENSOR_DATA.debug[2] + "]"} ], debug3_options);
    Flotr.draw(e_graph_debug4, [
        {data: debug_data[3], label: "debug4 [" + SENSOR_DATA.debug[3] + "]"} ], debug4_options);

    samples_debug_i++;
}

