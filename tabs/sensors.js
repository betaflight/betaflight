function initSensorData(){
    for (var i = 0; i < 3; i++) {
        SENSOR_DATA.accelerometer[i] = 0;
        SENSOR_DATA.gyroscope[i] = 0;
        SENSOR_DATA.magnetometer[i] = 0;
        SENSOR_DATA.debug[i] = 0;
    }
}

function initDataArray(length) {
    var data = new Array(length);
    for (var i = 0; i < length; i++) {
        data[i] = new Array();
        for (var j = 0; j <= 300; j++) {
            data[i].push([j, 0]);
        }
        data[i].min = -1;
        data[i].max = 1;
    }
    return data;
}

function addSampleToData(data, sampleNumber, sensorData) {
    for (var i = 0; i < data.length; i++) {
        var dataPoint = sensorData[i];
        data[i].push([sampleNumber, dataPoint]);
        if (dataPoint < data[i].min) {
            data[i].min = dataPoint;
        }
        if (dataPoint > data[i].max) {
            data[i].max = dataPoint;
        }
    }
    while (data[0].length > 300) {
        for (i = 0; i < data.length; i++) {
            data[i].shift();
        }
    }
    return sampleNumber + 1;
}

function initGraphHelpers(selector, sampleNumber, heightDomain) {
    var margin = {top: 20, right: 20, bottom: 10, left: 40};
    var width = 900 - margin.left - margin.right;
    var height = 120 - margin.top - margin.bottom;

    var helpers = {selector: selector, dynamicHeightDomain: !heightDomain};

    helpers.widthScale = d3.scale.linear().
        domain([(sampleNumber - 299), sampleNumber]).
        range([0, width]);

    helpers.heightScale = d3.scale.linear().
        domain(heightDomain || [1, -1]).
        range([height, 0]);

    helpers.xGrid = d3.svg.axis().
        scale(helpers.widthScale).
        orient("bottom").
        ticks(5).
        tickSize(-height, 0, 0).
        tickFormat("");

    helpers.yGrid = d3.svg.axis().
        scale(helpers.heightScale).
        orient("left").
        ticks(5).
        tickSize(-width, 0, 0).
        tickFormat("");

    helpers.xAxis = d3.svg.axis().
        scale(helpers.widthScale).
        ticks(5).
        orient("bottom").
        tickFormat(function(d) {return d;});

    helpers.yAxis = d3.svg.axis().
        scale(helpers.heightScale).
        ticks(5).
        orient("left").
        tickFormat(function(d) {return d;});

    helpers.line = d3.svg.line().
        x(function(d) { return helpers.widthScale(d[0]); }).
        y(function(d) { return helpers.heightScale(d[1]); });

    return helpers;
}

function drawGraph(graphHelpers, data, sampleNumber) {
    svg = d3.select(graphHelpers.selector);

    if (graphHelpers.dynamicHeightDomain) {
        var limits = [];
        $.each(data, function(idx, datum) {
            limits.push(datum.min);
            limits.push(datum.max);
        });
        graphHelpers.heightScale.domain(d3.extent(limits)).ticks(5);
    }
    graphHelpers.widthScale.domain([(sampleNumber - 299), sampleNumber]);

    svg.select(".x.grid").call(graphHelpers.xGrid);
    svg.select(".y.grid").call(graphHelpers.yGrid);
    svg.select(".x.axis").call(graphHelpers.xAxis);
    svg.select(".y.axis").call(graphHelpers.yAxis);

    var group = svg.select("g.data");
    var lines = group.selectAll("path").data(data, function(d, i) { return i; });
    var newLines = lines.enter().append("path").attr("class", "line");
    lines.attr('d', graphHelpers.line);
    svg.selectAll('.legend .item .value').data(data, function(d, i){ return i; }).
        text(function(d) { return "[ " + d[d.length - 1][1].toFixed(2) + " ]"; });
}

function tab_initialize_sensors() {
    ga_tracker.sendAppView('Sensor Page');
    GUI.active_tab = 'sensors';

    $('#content').load("./tabs/sensors.html", function load_html() {
        // Always start with default/empty sensor data array, clean slate all
        initSensorData();

        // Setup variables
        var samples_gyro_i = 300;
        var samples_accel_i = 300;
        var samples_mag_i = 300;
        var samples_baro_i = 300;
        var samples_debug_i = 300;

        var gyro_data = initDataArray(3);
        var accel_data = initDataArray(3);
        var mag_data = initDataArray(3);
        var baro_data = initDataArray(1);
        var debug_data = [
            initDataArray(1),
            initDataArray(1),
            initDataArray(1),
            initDataArray(1)
        ];

        var gyroHelpers = initGraphHelpers('#gyro', samples_gyro_i, [2000, -2000]);
        var accelHelpers = initGraphHelpers('#accel', samples_accel_i, [2, -2]);
        var magHelpers = initGraphHelpers('#mag', samples_mag_i);
        var baroHelpers = initGraphHelpers('#baro', samples_baro_i);
        var debugHelpers = [
            initGraphHelpers('#debug1', samples_debug_i),
            initGraphHelpers('#debug2', samples_debug_i),
            initGraphHelpers('#debug3', samples_debug_i),
            initGraphHelpers('#debug4', samples_debug_i)
        ];

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
                'gyro':   parseInt($('.tab-sensors select').eq(0).val(), 10),
                'accel':  parseInt($('.tab-sensors select').eq(1).val(), 10),
                'mag':    parseInt($('.tab-sensors select').eq(2).val(), 10),
                'baro':   parseInt($('.tab-sensors select').eq(3).val(), 10),
                'debug':  parseInt($('.tab-sensors select').eq(4).val(), 10)
            };

            // handling of "data pulling" is a little bit funky here, as MSP_RAW_IMU contains values for gyro/accel/mag but not baro
            // this means that setting a slower refresh rate on any of the attributes would have no effect
            // what we will do instead is = determinate the fastest refresh rate for those 3 attributes, use that as a "polling rate"
            // and use the "slower" refresh rates only for re-drawing the graphs (to save resources/computing power)
            var fastest = d3.min([rates.gyro, rates.accel, rates.mag]);

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
                samples_gyro_i = addSampleToData(gyro_data, samples_gyro_i, SENSOR_DATA.gyroscope);
                drawGraph(gyroHelpers, gyro_data, samples_gyro_i);

                samples_accel_i = addSampleToData(accel_data, samples_accel_i, SENSOR_DATA.accelerometer);
                drawGraph(accelHelpers, accel_data, samples_accel_i);

                samples_mag_i = addSampleToData(mag_data, samples_mag_i, SENSOR_DATA.magnetometer);
                drawGraph(magHelpers, mag_data, samples_mag_i);
            }

            function update_altitude_graph() {
                samples_baro_i = addSampleToData(baro_data, samples_baro_i, [SENSOR_DATA.altitude]);
                drawGraph(baroHelpers, baro_data, samples_baro_i);
            }

            function update_debug_graphs() {
                for (var i = 0; i < 4; i++) {
                    addSampleToData(debug_data[i], samples_debug_i, [SENSOR_DATA.debug[i]]);
                    drawGraph(debugHelpers[i], debug_data[i], samples_debug_i);
                }
                samples_debug_i++;
            }
        });
    });
}
