'use strict';

TABS.motors = {};
TABS.motors.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'motors') {
        GUI.active_tab = 'motors';
        googleAnalytics.sendAppView('Motors');
    }

    function get_motor_data() {
        MSP.send_message(MSP_codes.MSP_MOTOR, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/motors.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_MISC, false, false, get_motor_data);

    function initSensorData() {
        for (var i = 0; i < 3; i++) {
            SENSOR_DATA.accelerometer[i] = 0;
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

    var margin = {top: 20, right: 30, bottom: 10, left: 20};
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
            .x(function (d) { return helpers.widthScale(d[0]); })
            .y(function (d) { return helpers.heightScale(d[1]); });

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

    function process_html() {
        // translate to user-selected language
        localize();

        // Always start with default/empty sensor data array, clean slate all
        initSensorData();

        // Setup variables
        var samples_accel_i = 0,
            accel_data = initDataArray(3),
            accelHelpers = initGraphHelpers('#accel', samples_accel_i, [-2, 2]),
            accel_max_read = [0, 0, 0],
            accel_offset = [0, 0, 0],
            accel_offset_established = false;

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
        chrome.storage.local.get('motors_tab_accel_settings', function (result) {
            if (result.motors_tab_accel_settings) {
                $('.tab-motors select[name="accel_refresh_rate"]').val(result.motors_tab_accel_settings.rate);
                $('.tab-motors select[name="accel_scale"]').val(result.motors_tab_accel_settings.scale);

                // start polling data by triggering refresh rate change event
                $('.tab-motors .rate select:first').change();
            } else {
                // start polling immediatly (as there is no configuration saved in the storage)
                $('.tab-motors .rate select:first').change();
            }
        });

        $('.tab-motors .rate select, .tab-motors .scale select').change(function () {
            var rate = parseInt($('.tab-motors select[name="accel_refresh_rate"]').val(), 10);
            var scale = parseFloat($('.tab-motors select[name="accel_scale"]').val());

            // store current/latest refresh rates in the storage
            chrome.storage.local.set({'motors_tab_accel_settings': {'rate': rate, 'scale': scale}});

            accelHelpers = initGraphHelpers('#accel', samples_accel_i, [-scale, scale]);

            // timer initialization
            GUI.interval_kill_all(['motor_pull', 'status_pull']);

            GUI.interval_add('IMU_pull', function imu_data_pull() {
                MSP.send_message(MSP_codes.MSP_RAW_IMU, false, false, update_accel_graph);
            }, rate, true);

            function update_accel_graph() {
                if (!accel_offset_established) {
                    for (var i = 0; i < 3; i++) {
                        accel_offset[i] = SENSOR_DATA.accelerometer[i] * -1;
                    }

                    accel_offset_established = true;
                }

                var accel_with_offset = [
                    accel_offset[0] + SENSOR_DATA.accelerometer[0],
                    accel_offset[1] + SENSOR_DATA.accelerometer[1],
                    accel_offset[2] + SENSOR_DATA.accelerometer[2]
                ];

                updateGraphHelperSize(accelHelpers);
                samples_accel_i = addSampleToData(accel_data, samples_accel_i, accel_with_offset);
                drawGraph(accelHelpers, accel_data, samples_accel_i);

                raw_data_text_ements.x[0].text(accel_with_offset[0].toFixed(2) + ' (' + accel_max_read[0].toFixed(2) + ')');
                raw_data_text_ements.y[0].text(accel_with_offset[1].toFixed(2) + ' (' + accel_max_read[1].toFixed(2) + ')');
                raw_data_text_ements.z[0].text(accel_with_offset[2].toFixed(2) + ' (' + accel_max_read[2].toFixed(2) + ')');

                for (var i = 0; i < 3; i++) {
                    if (Math.abs(accel_with_offset[i]) > Math.abs(accel_max_read[i])) accel_max_read[i] = accel_with_offset[i];
                }
            }
        });

        $('a.reset_accel_max').click(function () {
            accel_max_read = [0, 0, 0];
            accel_offset_established = false;
        });

        var number_of_valid_outputs = (MOTOR_DATA.indexOf(0) > -1) ? MOTOR_DATA.indexOf(0) : 8;

        var motors_wrapper = $('.motors .bar-wrapper'),
            servos_wrapper = $('.servos .bar-wrapper');

        for (var i = 0; i < 8; i++) {
            motors_wrapper.append('\
                <div class="m-block motor-' + i + '">\
                    <div class="meter-bar">\
                        <div class="label"></div>\
                        <div class="indicator">\
                            <div class="label">\
                                <div class="label"></div>\
                            </div>\
                        </div>\
                    </div>\
                </div>\
            ');

            servos_wrapper.append('\
                <div class="m-block servo-' + i + '">\
                    <div class="meter-bar">\
                        <div class="label"></div>\
                        <div class="indicator">\
                            <div class="label">\
                                <div class="label"></div>\
                            </div>\
                        </div>\
                    </div>\
                </div>\
            ');
        }

        $('div.sliders input').prop('min', MISC.mincommand);
        $('div.sliders input').prop('max', MISC.maxthrottle);
        $('div.sliders input').val(MISC.mincommand);
        $('div.values li:not(:last)').text(MISC.mincommand);

        // UI hooks
        var buffering_set_motor = [],
            buffer_delay = false;
        $('div.sliders input:not(.master)').on('input', function () {
            var index = $(this).index(),
                buffer = [],
                i;

            $('div.values li').eq(index).text($(this).val());

            for (i = 0; i < 8; i++) {
                var val = parseInt($('div.sliders input').eq(i).val());

                buffer.push(lowByte(val));
                buffer.push(highByte(val));
            }

            buffering_set_motor.push(buffer);

            if (!buffer_delay) {
                buffer_delay = setTimeout(function () {
                    buffer = buffering_set_motor.pop();

                    MSP.send_message(MSP_codes.MSP_SET_MOTOR, buffer);

                    buffering_set_motor = [];
                    buffer_delay = false;
                }, 10);
            }
        });

        $('div.sliders input.master').on('input', function () {
            var val = $(this).val();

            $('div.sliders input:not(:disabled, :last)').val(val);
            $('div.values li:not(:last)').slice(0, number_of_valid_outputs).text(val);
            $('div.sliders input:not(:last):first').trigger('input');
        });

        $('div.notice input[type="checkbox"]').change(function () {
            if ($(this).is(':checked')) {
                $('div.sliders input').slice(0, number_of_valid_outputs).prop('disabled', false);

                // unlock master slider
                $('div.sliders input:last').prop('disabled', false);
            } else {
                // disable sliders / min max
                $('div.sliders input').prop('disabled', true);

                // change all values to default
                $('div.sliders input').val(MISC.mincommand);

                // trigger change event so values are sent to mcu
                $('div.sliders input').trigger('input');
            }
        });

        // check if motors are already spinning
        var motors_running = false;

        for (var i = 0; i < MOTOR_DATA.length; i++) {
            if (MOTOR_DATA[i] > MISC.mincommand) {
                motors_running = true;
                break;
            }
        }

        if (motors_running) {
            // motors are running, enable test mode and adjust sliders to current values
            $('div.notice input[type="checkbox"]').click();

            var sliders = $('div.sliders input:not(.master)');

            var master_value = MOTOR_DATA[0];
            for (var i = 0; i < MOTOR_DATA.length; i++) {
                if (MOTOR_DATA[i] > 0) {
                    sliders.eq(i).val(MOTOR_DATA[i]);

                    if (master_value != MOTOR_DATA[i]) {
                        master_value = false;
                    }
                }
            }

            // only fire events when all values are set
            sliders.trigger('input');

            // slide master slider if condition is valid
            if (master_value) {
                $('div.sliders input.master').val(master_value);
                $('div.sliders input.master').trigger('input');
            }
        }


        // data pulling functions used inside interval timer
        function get_motor_data() {
            MSP.send_message(MSP_codes.MSP_MOTOR, false, false, get_servo_data);
        }

        function get_servo_data() {
            MSP.send_message(MSP_codes.MSP_SERVO, false, false, update_ui);
        }

        var full_block_scale = MISC.maxthrottle - MISC.mincommand;
        function update_ui() {
            var block_height = $('div.m-block:first').height();

            for (var i = 0; i < MOTOR_DATA.length; i++) {
                var data = MOTOR_DATA[i] - MISC.mincommand,
                    margin_top = block_height - (data * (block_height / full_block_scale)).clamp(0, block_height),
                    height = (data * (block_height / full_block_scale)).clamp(0, block_height),
                    color = parseInt(data * 0.256);

                $('.motor-' + i + ' .label', motors_wrapper).text(MOTOR_DATA[i]);
                $('.motor-' + i + ' .indicator', motors_wrapper).css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
            }

            // servo indicators are still using old (not flexible block scale), it will be changed in the future accordingly
            for (var i = 0; i < SERVO_DATA.length; i++) {
                var data = SERVO_DATA[i] - 1000,
                    margin_top = block_height - (data * (block_height / 1000)).clamp(0, block_height),
                    height = (data * (block_height / 1000)).clamp(0, block_height),
                    color = parseInt(data * 0.256);

                $('.servo-' + i + ' .label', servos_wrapper).text(SERVO_DATA[i]);
                $('.servo-' + i + ' .indicator', servos_wrapper).css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
            }
        }

        // enable Motor data pulling
        GUI.interval_add('motor_pull', get_motor_data, 50, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.motors.cleanup = function (callback) {
    if (callback) callback();
};