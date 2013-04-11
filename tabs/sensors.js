function tab_initialize_sensors() {
    // Setup variables
    samples_i = 300;
    
    gyro_data = new Array(3);
    accel_data = new Array(3);
    mag_data = new Array(3);
    baro_data = new Array(1);
    
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
    }
    
    // plot specific stuff
    e_graph_gyro = document.getElementById("gyro");
    e_graph_accel = document.getElementById("accel");  
    e_graph_mag = document.getElementById("mag");
    e_graph_baro = document.getElementById("baro");  
    
    gyro_options = {
        title: "Gyroscope (deg/s)",
        shadowSize: 0,
        yaxis : {
            tickDecimals: 0,
            max: 150,
            min: -150
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
            tickDecimals: 0,
            max : 120,
            min : -120
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
        title: "Barometer (m)",
        shadowSize: 0,
        yaxis : {
            tickDecimals: 1,
            /*
            max : 1.5,
            min : -1.5
            */
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
    
    // start polling data
    timers.push(setInterval(sensor_array_pull, 50));  
}

function sensor_array_pull() {
    // push data to the main array
    gyro_data[0].push([samples_i, SENSOR_DATA.gyroscope[0]]);
    gyro_data[1].push([samples_i, SENSOR_DATA.gyroscope[1]]);
    gyro_data[2].push([samples_i, SENSOR_DATA.gyroscope[2]]);
    
    accel_data[0].push([samples_i, SENSOR_DATA.accelerometer[0]]);
    accel_data[1].push([samples_i, SENSOR_DATA.accelerometer[1]]);
    accel_data[2].push([samples_i, SENSOR_DATA.accelerometer[2]]);

    mag_data[0].push([samples_i, SENSOR_DATA.magnetometer[0]]);
    mag_data[1].push([samples_i, SENSOR_DATA.magnetometer[1]]);
    mag_data[2].push([samples_i, SENSOR_DATA.magnetometer[2]]);

    baro_data[0].push([samples_i, SENSOR_DATA.altitude]);
    
    // Remove old data from array
    while (gyro_data[0].length > 300) {
        gyro_data[0].shift();
        gyro_data[1].shift();
        gyro_data[2].shift();
        
        accel_data[0].shift();
        accel_data[1].shift();
        accel_data[2].shift(); 

        mag_data[0].shift();
        mag_data[1].shift();
        mag_data[2].shift();

        baro_data[0].shift();
    }    

    // Update graphs
    Flotr.draw(e_graph_gyro, [ 
        {data: gyro_data[0], label: "X - rate [" + SENSOR_DATA.gyroscope[0].toFixed(2) + "]"}, 
        {data: gyro_data[1], label: "Y - rate [" + SENSOR_DATA.gyroscope[1].toFixed(2) + "]"}, 
        {data: gyro_data[2], label: "Z - rate [" + SENSOR_DATA.gyroscope[2].toFixed(2) + "]"} ], gyro_options);  

    Flotr.draw(e_graph_accel, [ 
        {data: accel_data[1], label: "X - acceleration [" + SENSOR_DATA.accelerometer[0].toFixed(2) + "]"}, 
        {data: accel_data[0], label: "Y - acceleration [" + SENSOR_DATA.accelerometer[1].toFixed(2) + "]"}, 
        {data: accel_data[2], label: "Z - acceleration [" + SENSOR_DATA.accelerometer[2].toFixed(2) + "]"} ], accel_options);
        
    Flotr.draw(e_graph_mag, [ 
        {data: mag_data[1], label: "X - Ga [" + SENSOR_DATA.magnetometer[0].toFixed(2) + "]"}, 
        {data: mag_data[0], label: "Y - Ga [" + SENSOR_DATA.magnetometer[1].toFixed(2) + "]"}, 
        {data: mag_data[2], label: "Z - Ga [" + SENSOR_DATA.magnetometer[2].toFixed(2) + "]"} ], mag_options);    

    Flotr.draw(e_graph_baro, [ 
        {data: baro_data[0], label: "X - meters [" + SENSOR_DATA.altitude.toFixed(2) + "]"} ], baro_options);          
    
    samples_i++;
    
    // Request new data
    send_message(MSP_codes.MSP_RAW_IMU, MSP_codes.MSP_RAW_IMU);
    send_message(MSP_codes.MSP_ALTITUDE, MSP_codes.MSP_ALTITUDE);
}
