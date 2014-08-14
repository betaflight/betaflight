'use strict';

TABS.receiver = {};
TABS.receiver.initialize = function (callback) {
    GUI.active_tab_ref = this;
    GUI.active_tab = 'receiver';
    googleAnalytics.sendAppView('Receiver Page');

    MSP.send_message(MSP_codes.MSP_RC_TUNING, false, false, get_rc_data);

    function get_rc_data() {
        MSP.send_message(MSP_codes.MSP_RC, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/receiver.html", process_html);
    }

    function process_html() {
        // translate to user-selected language
        localize();

        // fill in data from RC_tuning
        $('.tunings .throttle input[name="mid"]').val(RC_tuning.throttle_MID.toFixed(2));
        $('.tunings .throttle input[name="expo"]').val(RC_tuning.throttle_EXPO.toFixed(2));

        $('.tunings .rate input[name="rate"]').val(RC_tuning.RC_RATE.toFixed(2));
        $('.tunings .rate input[name="expo"]').val(RC_tuning.RC_EXPO.toFixed(2));

        chrome.storage.local.get('rx_refresh_rate', function (result) {
            if (result.rx_refresh_rate) {
                $('select[name="rx_refresh_rate"]').val(result.rx_refresh_rate).change();
            } else {
                $('select[name="rx_refresh_rate"]').change(); // start with default value
            }
        });

        // generate bars
        var bar_names = ['Roll', 'Pitch', 'Yaw', 'Throttle'],
            bar_container = $('.tab-receiver .bars'),
            aux_index = 1;

        for (var i = 0; i < RC.active_channels; i++) {
            var name;
            if (i < bar_names.length) {
                name = bar_names[i];
            } else {
                name = 'AUX ' + aux_index++;
            }

            bar_container.append('\
                <ul>\
                    <li class="name">' + name + '</li>\
                    <li class="meter"><meter min="800" max="2200"></meter></li>\
                    <li class="value"></li>\
                </ul>\
                <div class="clear-both"></div>\
            ');
        }

        var meter_array = [];
        $('meter', bar_container).each(function () {
            meter_array.push($(this));
        });

        var meter_values_array = [];
        $('.value', bar_container).each(function () {
            meter_values_array.push($(this));
        });

        // UI Hooks
        // curves
        $('.tunings .throttle input').change(function () {
            setTimeout(function () {
                var mid = parseFloat($('.tunings .throttle input[name="mid"]').val());
                var expo = parseFloat($('.tunings .throttle input[name="expo"]').val());

                var throttle_curve = $('.throttle_curve canvas').get(0);
                var context = throttle_curve.getContext("2d");
                context.clearRect(0, 0, 220, 58);

                // math magic by englishman
                var midx = 220 * mid;
                var midxl = midx * 0.5;
                var midxr = (((220 - midx) * 0.5) + midx);
                var midy = 58 - (midx * (58 / 220));
                var midyl = 58 - ((58 - midy) * 0.5 *(expo + 1));
                var midyr = (midy / 2) * (expo + 1);

                context.beginPath();
                context.moveTo(0, 58);
                context.quadraticCurveTo(midxl, midyl, midx, midy);
                context.moveTo(midx, midy);
                context.quadraticCurveTo(midxr, midyr, 220, 0);

                context.lineWidth = 2;
                context.stroke();
            }, 0); // race condition, that should always trigger after all events are processed
        }).change();

        $('.tunings .rate input').change(function () {
            setTimeout(function () {
                var rate = parseFloat($('.tunings .rate input[name="rate"]').val());
                var expo = parseFloat($('.tunings .rate input[name="expo"]').val());

                var pitch_roll_curve = $('.pitch_roll_curve canvas').get(0);
                var context = pitch_roll_curve.getContext("2d");
                context.clearRect(0, 0, 220, 58);

                // math magic by englishman
                var ratey = 58 * rate;

                context.beginPath();
                context.moveTo(0, 58);
                context.quadraticCurveTo(110, 58 - ((ratey / 2) * (1 - expo)), 220, 58 - ratey);
                context.lineWidth = 2;
                context.stroke();
            }, 0); // race condition, that should always trigger after all events are processed
        }).change();

        $('a.refresh').click(function () {
            MSP.send_message(MSP_codes.MSP_RC_TUNING, false, false, function () {
                GUI.log(chrome.i18n.getMessage('receiverDataRefreshed'));

                // fill in data from RC_tuning
                $('.tunings .throttle input[name="mid"]').val(RC_tuning.throttle_MID.toFixed(2));
                $('.tunings .throttle input[name="expo"]').val(RC_tuning.throttle_EXPO.toFixed(2));

                $('.tunings .rate input[name="rate"]').val(RC_tuning.RC_RATE.toFixed(2));
                $('.tunings .rate input[name="expo"]').val(RC_tuning.RC_EXPO.toFixed(2));

                // update visual representation
                $('.tunings .throttle input').change();
                $('.tunings .rate input').change();
            });
        });

        $('a.update').click(function () {
            // catch RC_tuning changes
            RC_tuning.throttle_MID = parseFloat($('.tunings .throttle input[name="mid"]').val());
            RC_tuning.throttle_EXPO = parseFloat($('.tunings .throttle input[name="expo"]').val());

            RC_tuning.RC_RATE = parseFloat($('.tunings .rate input[name="rate"]').val());
            RC_tuning.RC_EXPO = parseFloat($('.tunings .rate input[name="expo"]').val());

            var RC_tuning_buffer_out = new Array();
            RC_tuning_buffer_out[0] = parseInt(RC_tuning.RC_RATE * 100, 10);
            RC_tuning_buffer_out[1] = parseInt(RC_tuning.RC_EXPO * 100, 10);
            RC_tuning_buffer_out[2] = parseInt(RC_tuning.roll_pitch_rate * 100, 10);
            RC_tuning_buffer_out[3] = parseInt(RC_tuning.yaw_rate * 100, 10);
            RC_tuning_buffer_out[4] = parseInt(RC_tuning.dynamic_THR_PID * 100, 10);
            RC_tuning_buffer_out[5] = parseInt(RC_tuning.throttle_MID * 100, 10);
            RC_tuning_buffer_out[6] = parseInt(RC_tuning.throttle_EXPO * 100, 10);

            // Send over the RC_tuning changes
            MSP.send_message(MSP_codes.MSP_SET_RC_TUNING, RC_tuning_buffer_out, false, save_to_eeprom);

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('receiverEepromSaved'));
                });
            }
        });

        $('select[name="rx_refresh_rate"]').change(function () {
            var plot_update_rate = parseInt($(this).val(), 10);

            // save update rate
            chrome.storage.local.set({'rx_refresh_rate': plot_update_rate});

            function get_rc_data() {
                MSP.send_message(MSP_codes.MSP_RC, false, false, update_ui);
            }

            // setup plot
            var RX_plot_data = new Array(RC.active_channels);
            for (var i = 0; i < RX_plot_data.length; i++) {
                RX_plot_data[i] = [];
            }

            var samples = 0,
                svg = d3.select("svg"),
                RX_plot_e = $('#RX_plot'),
                margin = {top: 20, right: 0, bottom: 10, left: 40},
                width, height, widthScale, heightScale;

            function update_receiver_plot_size() {
                width = RX_plot_e.width() - margin.left - margin.right;
                height = RX_plot_e.height() - margin.top - margin.bottom;

                widthScale.range([0, width]);
                heightScale.range([height, 0]);
            }

            function update_ui() {
                // update bars with latest data
                for (var i = 0; i < RC.active_channels; i++) {
                    meter_array[i].val(RC.channels[i]);
                    meter_values_array[i].text('[ ' + RC.channels[i] + ' ]');
                }

                // push latest data to the main array
                for (var i = 0; i < RC.active_channels; i++) {
                    RX_plot_data[i].push([samples, RC.channels[i]]);
                }

                // Remove old data from array
                while (RX_plot_data[0].length > 300) {
                    for (var i = 0; i < RX_plot_data.length; i++) {
                        RX_plot_data[i].shift();
                    }
                }

                // update required parts of the plot
                widthScale = d3.scale.linear().
                    domain([(samples - 299), samples]);

                heightScale = d3.scale.linear().
                    domain([800, 2200]);

                update_receiver_plot_size();

                var xGrid = d3.svg.axis().
                    scale(widthScale).
                    orient("bottom").
                    tickSize(-height, 0, 0).
                    tickFormat("");

                var yGrid = d3.svg.axis().
                    scale(heightScale).
                    orient("left").
                    tickSize(-width, 0, 0).
                    tickFormat("");

                var xAxis = d3.svg.axis().
                    scale(widthScale).
                    orient("bottom").
                    tickFormat(function (d) {return d;});

                var yAxis = d3.svg.axis().
                    scale(heightScale).
                    orient("left").
                    tickFormat(function (d) {return d;});

                var line = d3.svg.line().
                    x(function (d) {return widthScale(d[0]);}).
                    y(function (d) {return heightScale(d[1]);});

                svg.select(".x.grid").call(xGrid);
                svg.select(".y.grid").call(yGrid);
                svg.select(".x.axis").call(xAxis);
                svg.select(".y.axis").call(yAxis);

                var data = svg.select("g.data"),
                    lines = data.selectAll("path").data(RX_plot_data, function (d, i) {return i;}),
                    newLines = lines.enter().append("path").attr("class", "line");
                lines.attr('d', line);

                samples++;
            }

            // timer initialization
            GUI.interval_remove('receiver_pull');

            // enable RC data pulling
            GUI.interval_add('receiver_pull', get_rc_data, plot_update_rate, true);
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function () {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.receiver.cleanup = function (callback) {
    if (callback) callback();
};
