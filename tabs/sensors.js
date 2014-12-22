'use strict';

TABS.sensors = {};
TABS.sensors.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'sensors') {
        GUI.active_tab = 'sensors';
        googleAnalytics.sendAppView('Sensors');
    }

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

    var margin = {top: 20, right: 10, bottom: 10, left: 40};
    function updateGraphHelperSize(helpers) {
        helpers.width = helpers.targetElement.width() - margin.left - margin.right;
        helpers.height = helpers.targetElement.height() - margin.top - margin.bottom;

        helpers.widthScale.range([0, helpers.width]);
        helpers.heightScale.range([helpers.height, 0]);

        helpers.xGrid.tickSize(-helpers.height, 0, 0);
        helpers.yGrid.tickSize(-helpers.width, 0, 0);
    }

    function initGraphHelpers(selector, sampleNumber, heightDomain) {
        var helpers = {selector: selector, targetElement: $(selector), dynamicHeightDomain: !heightDomain};

        helpers.widthScale = d3.scale.linear()
            .clamp(true)
            .domain([(sampleNumber - 299), sampleNumber]);

        helpers.heightScale = d3.scale.linear()
            .clamp(true)
            .domain(heightDomain || [1, -1]);

        helpers.xGrid = d3.svg.axis();
        helpers.yGrid = d3.svg.axis();

        updateGraphHelperSize(helpers);

        helpers.xGrid
            .scale(helpers.widthScale)
            .orient("bottom")
            .ticks(5)
            .tickFormat("");

        helpers.yGrid
            .scale(helpers.heightScale)
            .orient("left")
            .ticks(5)
            .tickFormat("");

        helpers.xAxis = d3.svg.axis()
            .scale(helpers.widthScale)
            .ticks(5)
            .orient("bottom")
            .tickFormat(function (d) {return d;});

        helpers.yAxis = d3.svg.axis()
            .scale(helpers.heightScale)
            .ticks(5)
            .orient("left")
            .tickFormat(function (d) {return d;});

        helpers.line = d3.svg.line()
            .x(function (d) {return helpers.widthScale(d[0]);})
            .y(function (d) {return helpers.heightScale(d[1]);});

        return helpers;
    }

    function drawGraph(graphHelpers, data, sampleNumber) {
        var svg = d3.select(graphHelpers.selector);

        if (graphHelpers.dynamicHeightDomain) {
            var limits = [];
            $.each(data, function (idx, datum) {
                limits.push(datum.min);
                limits.push(datum.max);
            });
            graphHelpers.heightScale.domain(d3.extent(limits));
        }
        graphHelpers.widthScale.domain([(sampleNumber - 299), sampleNumber]);

        svg.select(".x.grid").call(graphHelpers.xGrid);
        svg.select(".y.grid").call(graphHelpers.yGrid);
        svg.select(".x.axis").call(graphHelpers.xAxis);
        svg.select(".y.axis").call(graphHelpers.yAxis);

        var group = svg.select("g.data");
        var lines = group.selectAll("path").data(data, function (d, i) {return i;});
        var newLines = lines.enter().append("path").attr("class", "line");
        lines.attr('d', graphHelpers.line);
    }

    function plot_gyro(enable) {
        if (enable) {
            $('.wrapper.gyro').show();
        } else {
            $('.wrapper.gyro').hide();
        }
    }

    function plot_accel(enable) {
        if (enable) {
            $('.wrapper.accel').show();
        } else {
            $('.wrapper.accel').hide();
        }
    }

    function plot_mag(enable) {
        if (enable) {
            $('.wrapper.mag').show();
        } else {
            $('.wrapper.mag').hide();
        }
    }

    function plot_baro(enable) {
        if (enable) {
            $('.wrapper.baro').show();
        } else {
            $('.wrapper.baro').hide();
        }
    }

    function plot_debug(enable) {
        if (enable) {
            $('.wrapper.debug').show();
        } else {
            $('.wrapper.debug').hide();
        }
    }

    $('#content').load("./tabs/sensors.html", function load_html() {
        // translate to user-selected language
        localize();

        // disable graphs for sensors that are missing
        var checkboxes = $('.tab-sensors .info .checkboxes input');
        if (!bit_check(CONFIG.activeSensors, 1)) { // baro
            checkboxes.eq(3).prop('disabled', true);
        }
        if (!bit_check(CONFIG.activeSensors, 2)) { // mag
            checkboxes.eq(2).prop('disabled', true);
        }

        $('.tab-sensors .info .checkboxes input').change(function () {
            var enable = $(this).prop('checked');
            var index = $(this).parent().index();

            switch (index) {
                case 0:
                    plot_gyro(enable);
                    break;
                case 1:
                    plot_accel(enable);
                    break;
                case 2:
                    plot_mag(enable);
                    break;
                case 3:
                    plot_baro(enable);
                    break;
                case 4:
                    plot_debug(enable);
                    break;
            }

            var checkboxes = [];
            $('.tab-sensors .info .checkboxes input').each(function () {
                checkboxes.push($(this).prop('checked'));
            });

            $('.tab-sensors .rate select:first').change();

            chrome.storage.local.set({'graphs_enabled': checkboxes});
        });

        chrome.storage.local.get('graphs_enabled', function (result) {
            if (result.graphs_enabled) {
                var checkboxes = $('.tab-sensors .info .checkboxes input');
                for (var i = 0; i < result.graphs_enabled.length; i++) {
                    checkboxes.eq(i).not(':disabled').prop('checked', result.graphs_enabled[i]).change();
                }
            } else {
                $('.tab-sensors .info input:lt(4):not(:disabled)').prop('checked', true).change();
            }
        });

        // Always start with default/empty sensor data array, clean slate all
        initSensorData();

        // Setup variables
        var samples_gyro_i = 0,
            samples_accel_i = 0,
            samples_mag_i = 0,
            samples_baro_i = 0,
            samples_debug_i = 0,
            gyro_data = initDataArray(3),
            accel_data = initDataArray(3),
            mag_data = initDataArray(3),
            baro_data = initDataArray(1),
            debug_data = [
            initDataArray(1),
            initDataArray(1),
            initDataArray(1),
            initDataArray(1)
        ];

        var gyroHelpers = initGraphHelpers('#gyro', samples_gyro_i, [-2000, 2000]);
        var accelHelpers = initGraphHelpers('#accel', samples_accel_i, [-2, 2]);
        var magHelpers = initGraphHelpers('#mag', samples_mag_i, [-1, 1]);
        var baroHelpers = initGraphHelpers('#baro', samples_baro_i);
        var debugHelpers = [
            initGraphHelpers('#debug1', samples_debug_i),
            initGraphHelpers('#debug2', samples_debug_i),
            initGraphHelpers('#debug3', samples_debug_i),
            initGraphHelpers('#debug4', samples_debug_i)
        ];

        var raw_data_text_ements = {
            x: [],
            y: [],
            z: []
        };
        $('.plot_control .x, .plot_control .y, .plot_control .z').each(function () {
            var el = $(this);
            if (el.hasClass('x')) {
                raw_data_text_ements.x.push(el);
            } else if (el.hasClass('y')) {
                raw_data_text_ements.y.push(el);
            } else {
                raw_data_text_ements.z.push(el);
            }
        });

        // set refresh speeds according to configuration saved in storage
        chrome.storage.local.get('sensor_settings', function (result) {
            if (result.sensor_settings) {
                $('.tab-sensors select[name="gyro_refresh_rate"]').val(result.sensor_settings.rates.gyro);
                $('.tab-sensors select[name="gyro_scale"]').val(result.sensor_settings.scales.gyro);

                $('.tab-sensors select[name="accel_refresh_rate"]').val(result.sensor_settings.rates.accel);
                $('.tab-sensors select[name="accel_scale"]').val(result.sensor_settings.scales.accel);

                $('.tab-sensors select[name="mag_refresh_rate"]').val(result.sensor_settings.rates.mag);
                $('.tab-sensors select[name="mag_scale"]').val(result.sensor_settings.scales.mag);

                $('.tab-sensors select[name="baro_refresh_rate"]').val(result.sensor_settings.rates.baro);
                $('.tab-sensors select[name="debug_refresh_rate"]').val(result.sensor_settings.rates.debug);

                // start polling data by triggering refresh rate change event
                $('.tab-sensors .rate select:first').change();
            } else {
                // start polling immediatly (as there is no configuration saved in the storage)
                $('.tab-sensors .rate select:first').change();
            }
        });

        $('.tab-sensors .rate select, .tab-sensors .scale select').change(function () {
            // if any of the select fields change value, all of the select values are grabbed
            // and timers are re-initialized with the new settings
            var rates = {
                'gyro':   parseInt($('.tab-sensors select[name="gyro_refresh_rate"]').val(), 10),
                'accel':  parseInt($('.tab-sensors select[name="accel_refresh_rate"]').val(), 10),
                'mag':    parseInt($('.tab-sensors select[name="mag_refresh_rate"]').val(), 10),
                'baro':   parseInt($('.tab-sensors select[name="baro_refresh_rate"]').val(), 10),
                'debug':  parseInt($('.tab-sensors select[name="debug_refresh_rate"]').val(), 10)
            };

            var scales = {
                'gyro':  parseFloat($('.tab-sensors select[name="gyro_scale"]').val()),
                'accel': parseFloat($('.tab-sensors select[name="accel_scale"]').val()),
                'mag':   parseFloat($('.tab-sensors select[name="mag_scale"]').val())
            };

            // handling of "data pulling" is a little bit funky here, as MSP_RAW_IMU contains values for gyro/accel/mag but not baro
            // this means that setting a slower refresh rate on any of the attributes would have no effect
            // what we will do instead is = determinate the fastest refresh rate for those 3 attributes, use that as a "polling rate"
            // and use the "slower" refresh rates only for re-drawing the graphs (to save resources/computing power)
            var fastest = d3.min([rates.gyro, rates.accel, rates.mag]);

            // store current/latest refresh rates in the storage
            chrome.storage.local.set({'sensor_settings': {'rates': rates, 'scales': scales}});

            // re-initialize domains with new scales
            gyroHelpers = initGraphHelpers('#gyro', samples_gyro_i, [-scales.gyro, scales.gyro]);
            accelHelpers = initGraphHelpers('#accel', samples_accel_i, [-scales.accel, scales.accel]);
            magHelpers = initGraphHelpers('#mag', samples_mag_i, [-scales.mag, scales.mag]);

            // fetch currently enabled plots
            var checkboxes = [];
            $('.tab-sensors .info .checkboxes input').each(function () {
                checkboxes.push($(this).prop('checked'));
            });

            // timer initialization
            GUI.interval_kill_all(['status_pull']);

            // data pulling timers
            if (checkboxes[0] || checkboxes[1] || checkboxes[2]) {
                GUI.interval_add('IMU_pull', function imu_data_pull() {
                    MSP.send_message(MSP_codes.MSP_RAW_IMU, false, false, update_imu_graphs);
                }, fastest, true);
            }

            if (checkboxes[3]) {
                GUI.interval_add('altitude_pull', function altitude_data_pull() {
                    MSP.send_message(MSP_codes.MSP_ALTITUDE, false, false, update_altitude_graph);
                }, rates.baro, true);
            }

            if (checkboxes[4]) {
                GUI.interval_add('debug_pull', function debug_data_pull() {
                    MSP.send_message(MSP_codes.MSP_DEBUG, false, false, update_debug_graphs);
                }, rates.debug, true);
            }

            function update_imu_graphs() {
                if (checkboxes[0]) {
                    updateGraphHelperSize(gyroHelpers);

                    samples_gyro_i = addSampleToData(gyro_data, samples_gyro_i, SENSOR_DATA.gyroscope);
                    drawGraph(gyroHelpers, gyro_data, samples_gyro_i);
                    raw_data_text_ements.x[0].text(SENSOR_DATA.gyroscope[0].toFixed(2));
                    raw_data_text_ements.y[0].text(SENSOR_DATA.gyroscope[1].toFixed(2));
                    raw_data_text_ements.z[0].text(SENSOR_DATA.gyroscope[2].toFixed(2));
                }

                if (checkboxes[1]) {
                    updateGraphHelperSize(accelHelpers);

                    samples_accel_i = addSampleToData(accel_data, samples_accel_i, SENSOR_DATA.accelerometer);
                    drawGraph(accelHelpers, accel_data, samples_accel_i);
                    raw_data_text_ements.x[1].text(SENSOR_DATA.accelerometer[0].toFixed(2));
                    raw_data_text_ements.y[1].text(SENSOR_DATA.accelerometer[1].toFixed(2));
                    raw_data_text_ements.z[1].text(SENSOR_DATA.accelerometer[2].toFixed(2));
                }

                if (checkboxes[2]) {
                    updateGraphHelperSize(magHelpers);

                    samples_mag_i = addSampleToData(mag_data, samples_mag_i, SENSOR_DATA.magnetometer);
                    drawGraph(magHelpers, mag_data, samples_mag_i);
                    raw_data_text_ements.x[2].text(SENSOR_DATA.magnetometer[0].toFixed(2));
                    raw_data_text_ements.y[2].text(SENSOR_DATA.magnetometer[1].toFixed(2));
                    raw_data_text_ements.z[2].text(SENSOR_DATA.magnetometer[2].toFixed(2));
                }
            }

            function update_altitude_graph() {
                updateGraphHelperSize(baroHelpers);

                samples_baro_i = addSampleToData(baro_data, samples_baro_i, [SENSOR_DATA.altitude]);
                drawGraph(baroHelpers, baro_data, samples_baro_i);
                raw_data_text_ements.x[3].text(SENSOR_DATA.altitude.toFixed(2));
            }

            function update_debug_graphs() {
                for (var i = 0; i < 4; i++) {
                    updateGraphHelperSize(debugHelpers[i]);

                    addSampleToData(debug_data[i], samples_debug_i, [SENSOR_DATA.debug[i]]);
                    drawGraph(debugHelpers[i], debug_data[i], samples_debug_i);
                    raw_data_text_ements.x[4 + i].text(SENSOR_DATA.debug[i]);
                }
                samples_debug_i++;
            }
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    });
};

TABS.sensors.cleanup = function (callback) {
    serial.emptyOutputBuffer();

    if (callback) callback();
};
