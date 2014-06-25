function tab_initialize_motor_outputs() {
    ga_tracker.sendAppView('Motor Outputs Page');
    GUI.active_tab = 'motor_outputs';

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

    var margin = {top: 20, right: 10, bottom: 10, left: 20};
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
            .tickFormat(function(d) {return d;});

        helpers.yAxis = d3.svg.axis()
            .scale(helpers.heightScale)
            .ticks(5)
            .orient("left")
            .tickFormat(function(d) {return d;});

        helpers.line = d3.svg.line()
            .x(function(d) { return helpers.widthScale(d[0]); })
            .y(function(d) { return helpers.heightScale(d[1]); });

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
            graphHelpers.heightScale.domain(d3.extent(limits));
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
    }

    MSP.send_message(MSP_codes.MSP_MISC, false, false, get_motor_data);

    function get_motor_data() {
        MSP.send_message(MSP_codes.MSP_MOTOR, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/motor_outputs.html", process_html);
    }

    function process_html() {
        // translate to user-selected language
        localize();

        // Always start with default/empty sensor data array, clean slate all
        initSensorData();

        // Setup variables
        var samples_accel_i = 0;
        var accel_data = initDataArray(3);
        var accelHelpers = initGraphHelpers('#accel', samples_accel_i, [-2, 2]);

        var raw_data_text_ements = {
            x: [],
            y: [],
            z: [],
        };
        $('.plot_control .x, .plot_control .y, .plot_control .z').each(function() {
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
        chrome.storage.local.get('motors_tab_accel_settings', function(result) {
            if (result.motors_tab_accel_settings) {
                $('.tab-motor_outputs select[name="accel_refresh_rate"]').val(result.motors_tab_accel_settings.rate);
                $('.tab-motor_outputs select[name="accel_scale"]').val(result.motors_tab_accel_settings.scale);

                // start polling data by triggering refresh rate change event
                $('.tab-motor_outputs .rate select:first').change();
            } else {
                // start polling immediatly (as there is no configuration saved in the storage)
                $('.tab-motor_outputs .rate select:first').change();
            }
        });

        $('.tab-motor_outputs .rate select, .tab-motor_outputs .scale select').change(function() {
            var rate = parseInt($('.tab-motor_outputs select[name="accel_refresh_rate"]').val(), 10);
            var scale = parseFloat($('.tab-motor_outputs select[name="accel_scale"]').val());

            // store current/latest refresh rates in the storage
            chrome.storage.local.set({'motors_tab_accel_settings': {'rate': rate, 'scale': scale}});

            accelHelpers = initGraphHelpers('#accel', samples_accel_i, [-scale, scale]);

            // timer initialization
            GUI.interval_kill_all(['motor_pull', 'status_pull']);

            GUI.interval_add('IMU_pull', function imu_data_pull() {
                MSP.send_message(MSP_codes.MSP_RAW_IMU, false, false, update_accel_graph);
            }, rate, true);

            function update_accel_graph() {
                updateGraphHelperSize(accelHelpers);

                samples_accel_i = addSampleToData(accel_data, samples_accel_i, SENSOR_DATA.accelerometer);
                drawGraph(accelHelpers, accel_data, samples_accel_i);
                raw_data_text_ements.x[0].text(SENSOR_DATA.accelerometer[0].toFixed(2));
                raw_data_text_ements.y[0].text(SENSOR_DATA.accelerometer[1].toFixed(2));
                raw_data_text_ements.z[0].text(SENSOR_DATA.accelerometer[2].toFixed(2));
            }
        });

        // if CAP_DYNBALANCE is true
        if (bit_check(CONFIG.capability, 2)) {
            $('div.motor_testing').show();
        }

        var number_of_valid_outputs = (MOTOR_DATA.indexOf(0) > -1) ? MOTOR_DATA.indexOf(0) : 8;

        $('input.min').val(MISC.mincommand);
        $('input.max').val(MISC.maxthrottle);


        $('div.sliders input').prop('min', MISC.mincommand);
        $('div.sliders input').prop('max', MISC.maxthrottle);
        $('div.sliders input').val(MISC.mincommand);
        $('div.values li:not(:last)').html(MISC.mincommand);

        // UI hooks
        $('div.sliders input:not(.master)').on('input', function() {
            var index = $(this).index();

            $('div.values li').eq(index).html($(this).val());

            // send data to mcu
            var buffer_out = [];

            for (var i = 0; i < 8; i++) {
                var val = parseInt($('div.sliders input').eq(i).val());

                buffer_out.push(lowByte(val));
                buffer_out.push(highByte(val));
            }

            MSP.send_message(MSP_codes.MSP_SET_MOTOR, buffer_out);
        });

        $('div.sliders input.master').on('input', function() {
            var val = $(this).val();

            $('div.sliders input:not(:disabled, :last)').val(val);
            $('div.values li:not(:last)').slice(0, number_of_valid_outputs).html(val);
            $('div.sliders input:not(:last):first').trigger('input');
        });

        $('div.notice input[type="checkbox"]').change(function() {
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

            for (var i = 0; i < MOTOR_DATA.length; i++) {
                if (MOTOR_DATA[i] > 0) {
                    sliders.eq(i).val(MOTOR_DATA[i]);
                }
            }

            // only fire events when all values are set
            sliders.trigger('input');
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
                var data = MOTOR_DATA[i] - MISC.mincommand;
                var margin_top = block_height - (data * (block_height / full_block_scale));
                var height = (data * (block_height / full_block_scale));
                var color = parseInt(data * 0.256);

                $('.motor-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
            }

            // servo indicators are still using old (not flexible block scale), it will be changed in the future accordingly
            for (var i = 0; i < SERVO_DATA.length; i++) {
                var data = SERVO_DATA[i] - 1000;
                var margin_top = block_height - (data * (block_height / 1000));
                var height = (data * (block_height / 1000));
                var color = parseInt(data * 0.256);

                $('.servo-' + i + ' .indicator').css({'margin-top' : margin_top + 'px', 'height' : height + 'px', 'background-color' : 'rgb(' + color + ',0,0)'});
            }
        }

        // enable Motor data pulling
        GUI.interval_add('motor_pull', get_motor_data, 50, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function get_status_data() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);
    }
}