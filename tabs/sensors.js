function tab_initialize_sensors() {
    ga_tracker.sendAppView('Sensor Page');
    GUI.active_tab = 'sensors';

    $('#content').load("./tabs/sensors.html", function load_html() {
        // Always start with default/empty sensor data array, clean slate all
        for (var i = 0; i < 3; i++) SENSOR_DATA.accelerometer[i] = 0;
        for (var i = 0; i < 3; i++) SENSOR_DATA.gyroscope[i] = 0;
        for (var i = 0; i < 3; i++) SENSOR_DATA.magnetometer[i] = 0;
        for (var i = 0; i < 4; i++) SENSOR_DATA.debug[i] = 0;

        // Setup variables
        var samples_gyro_i = 300;
        var samples_accel_i = 300;
        var samples_mag_i = 300;
        var samples_baro_i = 300;
        var samples_debug_i = 300;

        var gyro_data = new Array(3);
        var accel_data = new Array(3);
        var mag_data = new Array(3);
        var baro_data = new Array(1);
        var debug_data = new Array(4);

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
        var e_graph_gyro = document.getElementById("gyro");
        var e_graph_accel = document.getElementById("accel");
        var e_graph_mag = document.getElementById("mag");
        var e_graph_baro = document.getElementById("baro");
        var e_graph_debug1 = document.getElementById("debug1");
        var e_graph_debug2 = document.getElementById("debug2");
        var e_graph_debug3 = document.getElementById("debug3");
        var e_graph_debug4 = document.getElementById("debug4");

        var gyro_options = {
            title: "Gyroscope (deg/s)",
            shadowSize: 0,
            yaxis : {
                tickDecimals: 1,
                max : 2000,
                min: -2000
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
        };

        var accel_options = {
            title: "Accelerometer (g)",
            shadowSize: 0,
            yaxis : {
                tickDecimals: 1,
                max : 2,
                min : -2
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
        };

        var mag_options = {
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
        };

        var baro_options = {
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
        };

        var debug1_options = {
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
        };

        var debug2_options = {};
        for (var key in debug1_options) debug2_options[key] = debug1_options[key];
        debug2_options.title = "Debug2";

        var debug3_options = {};
        for (var key in debug1_options) debug3_options[key] = debug1_options[key];
        debug3_options.title = "Debug3";

        var debug4_options = {};
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
                'gyro':   parseInt($('.tab-sensors select').eq(0).val()),
                'accel':  parseInt($('.tab-sensors select').eq(1).val()),
                'mag':    parseInt($('.tab-sensors select').eq(2).val()),
                'baro':   parseInt($('.tab-sensors select').eq(3).val()),
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

            // store current/latest refresh rates in the storage
            chrome.storage.local.set({'sensor_refresh_rates': rates});

            // timer initialization
            GUI.interval_kill_all();

            // data pulling timers

            // status data pulled via separate timer with static speed
            GUI.interval_add('status_pull', function() {
                send_message(MSP_codes.MSP_STATUS);
            }, 250, true);

            GUI.interval_add('IMU_pull', function imu_data_pull() {
                send_message(MSP_codes.MSP_RAW_IMU, false, false, update_imu_graphs);
            }, fastest, true);

            GUI.interval_add('altitude_pull', function altitude_data_pull() {
                send_message(MSP_codes.MSP_ALTITUDE, false, false, update_altitude_graph);
            }, rates.baro, true);

            GUI.interval_add('debug_pull', function debug_data_pull() {
                send_message(MSP_codes.MSP_DEBUG, false, false, update_debug_graphs);
            }, rates.debug, true);

            function update_imu_graphs() {
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
                    {data: mag_data[1], label: "X - gauss [" + SENSOR_DATA.magnetometer[0].toFixed(2) + "]"},
                    {data: mag_data[0], label: "Y - gauss [" + SENSOR_DATA.magnetometer[1].toFixed(2) + "]"},
                    {data: mag_data[2], label: "Z - gauss [" + SENSOR_DATA.magnetometer[2].toFixed(2) + "]"} ], mag_options);

                samples_mag_i++;
            }

            function update_altitude_graph() {
                baro_data[0].push([samples_baro_i, SENSOR_DATA.altitude]);

                // Remove old data from array
                while (baro_data[0].length > 300) {
                    baro_data[0].shift();
                }

                Flotr.draw(e_graph_baro, [
                    {data: baro_data[0], label: "Meters [" + SENSOR_DATA.altitude.toFixed(2) + "]"} ], baro_options);

                samples_baro_i++;
            }

            function update_debug_graphs() {
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
        });
    });
}