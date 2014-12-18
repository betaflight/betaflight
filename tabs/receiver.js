'use strict';

TABS.receiver = {};
TABS.receiver.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'receiver') {
        GUI.active_tab = 'receiver';
        googleAnalytics.sendAppView('Receiver');
    }

    function get_misc_data() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, get_rc_data);
    }

    function get_rc_data() {
        MSP.send_message(MSP_codes.MSP_RC, false, false, get_rc_map);
    }

    function get_rc_map() {
        MSP.send_message(MSP_codes.MSP_RCMAP, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/receiver.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_RC_TUNING, false, false, get_misc_data);

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
                    <li class="meter">\
                        <div class="meter-bar">\
                            <div class="label"></div>\
                            <div class="fill">\
                                <div class="label"></div>\
                            </div>\
                        </div>\
                    </li>\
                </ul>\
            ');
        }

        // we could probably use min and max throttle for the range, will see
        var meter_scale = {
            'min': 800,
            'max': 2200
        };

        var meter_fill_array = [];
        $('.meter .fill', bar_container).each(function () {
            meter_fill_array.push($(this));
        });

        var meter_label_array = [];
        $('.meter', bar_container).each(function () {
            meter_label_array.push($('.label' , this));
        });

        // correct inner label margin on window resize (i don't know how we could do this in css)
        self.resize = function () {
            var containerWidth = $('.meter:first', bar_container).width(),
                labelWidth = $('.meter .label:first', bar_container).width(),
                margin = (containerWidth / 2) - (labelWidth / 2);

            for (var i = 0; i < meter_label_array.length; i++) {
                meter_label_array[i].css('margin-left', margin);
            }
        };

        $(window).on('resize', self.resize).resize(); // trigger so labels get correctly aligned on creation

        // handle rcmap & rssi aux channel
        var RC_MAP_Letters = ['A', 'E', 'R', 'T', '1', '2', '3', '4'];

        var strBuffer = [];
        for (var i = 0; i < RC_MAP.length; i++) {
            strBuffer[RC_MAP[i]] = RC_MAP_Letters[i];
        }

        // reconstruct
        var str = strBuffer.join('');

        // set current value
        $('input[name="rcmap"]').val(str);

        // validation / filter
        var last_valid = str;

        $('input[name="rcmap"]').on('input', function () {
            var val = $(this).val();

            // limit length to max 8
            if (val.length > 8) {
                val = val.substr(0, 8);
                $(this).val(val);
            }
        });

        $('input[name="rcmap"]').focusout(function () {
            var val = $(this).val(),
                strBuffer = val.split(''),
                duplicityBuffer = [];

            if (val.length != 8) {
                $(this).val(last_valid);
                return false;
            }

            // check if characters inside are all valid, also check for duplicity
            for (var i = 0; i < val.length; i++) {
                if (RC_MAP_Letters.indexOf(strBuffer[i]) < 0) {
                    $(this).val(last_valid);
                    return false;
                }

                if (duplicityBuffer.indexOf(strBuffer[i]) < 0) {
                    duplicityBuffer.push(strBuffer[i]);
                } else {
                    $(this).val(last_valid);
                    return false;
                }
            }
        });

        // handle helper
        $('select[name="rcmap_helper"]').val(0); // go out of bounds
        $('select[name="rcmap_helper"]').change(function () {
            $('input[name="rcmap"]').val($(this).val());
        });

        // rssi aux
        $('select[name="rssi_aux_channel"]').val(MISC.rssi_aux_channel);

        // UI Hooks
        // curves
        $('.tunings .throttle input').on('input change', function () {
            setTimeout(function () { // let global validation trigger and adjust the values first
                var throttleMidE = $('.tunings .throttle input[name="mid"]'),
                    throttleExpoE = $('.tunings .throttle input[name="expo"]'),
                    mid = parseFloat(throttleMidE.val()),
                    expo = parseFloat(throttleExpoE.val()),
                    throttle_curve = $('.throttle_curve canvas').get(0),
                    context = throttle_curve.getContext("2d");

                // local validation to deal with input event
                if (mid >= parseFloat(throttleMidE.prop('min')) &&
                    mid <= parseFloat(throttleMidE.prop('max')) &&
                    expo >= parseFloat(throttleExpoE.prop('min')) &&
                    expo <= parseFloat(throttleExpoE.prop('max'))) {
                    // continue
                } else {
                    return;
                }

                // math magic by englishman
                var midx = 220 * mid,
                    midxl = midx * 0.5,
                    midxr = (((220 - midx) * 0.5) + midx),
                    midy = 58 - (midx * (58 / 220)),
                    midyl = 58 - ((58 - midy) * 0.5 *(expo + 1)),
                    midyr = (midy / 2) * (expo + 1);

                // draw
                context.clearRect(0, 0, 220, 58);
                context.beginPath();
                context.moveTo(0, 58);
                context.quadraticCurveTo(midxl, midyl, midx, midy);
                context.moveTo(midx, midy);
                context.quadraticCurveTo(midxr, midyr, 220, 0);
                context.lineWidth = 2;
                context.stroke();
            }, 0);
        }).trigger('input');

        $('.tunings .rate input').on('input change', function () {
            setTimeout(function () { // let global validation trigger and adjust the values first
                var rateE = $('.tunings .rate input[name="rate"]'),
                    expoE = $('.tunings .rate input[name="expo"]'),
                    rate = parseFloat(rateE.val()),
                    expo = parseFloat(expoE.val()),
                    pitch_roll_curve = $('.pitch_roll_curve canvas').get(0),
                    context = pitch_roll_curve.getContext("2d");

                // local validation to deal with input event
                if (rate >= parseFloat(rateE.prop('min')) &&
                    rate <= parseFloat(rateE.prop('max')) &&
                    expo >= parseFloat(expoE.prop('min')) &&
                    expo <= parseFloat(expoE.prop('max'))) {
                    // continue
                } else {
                    return;
                }

                // math magic by englishman
                var ratey = 58 * rate;

                // draw
                context.clearRect(0, 0, 220, 58);
                context.beginPath();
                context.moveTo(0, 58);
                context.quadraticCurveTo(110, 58 - ((ratey / 2) * (1 - expo)), 220, 58 - ratey);
                context.lineWidth = 2;
                context.stroke();
            }, 0);
        }).trigger('input');

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

            // catch rc map
            var RC_MAP_Letters = ['A', 'E', 'R', 'T', '1', '2', '3', '4'];
            var strBuffer = $('input[name="rcmap"]').val().split('');

            for (var i = 0; i < RC_MAP.length; i++) {
                RC_MAP[i] = strBuffer.indexOf(RC_MAP_Letters[i]);
            }

            // catch rssi aux
            MISC.rssi_aux_channel = parseInt($('select[name="rssi_aux_channel"]').val());

            function save_rc_map() {
                MSP.send_message(MSP_codes.MSP_SET_RCMAP, MSP.crunch(MSP_codes.MSP_SET_RCMAP), false, save_misc);
            }

            function save_misc() {
                MSP.send_message(MSP_codes.MSP_SET_MISC, MSP.crunch(MSP_codes.MSP_SET_MISC), false, save_to_eeprom);
            }

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('receiverEepromSaved'));
                });
            }

            MSP.send_message(MSP_codes.MSP_SET_RC_TUNING, MSP.crunch(MSP_codes.MSP_SET_RC_TUNING), false, save_rc_map);
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
                    meter_fill_array[i].css('width', ((RC.channels[i] - meter_scale.min) / (meter_scale.max - meter_scale.min) * 100).clamp(0, 100) + '%');
                    meter_label_array[i].text(RC.channels[i]);
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
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.receiver.cleanup = function (callback) {
    $(window).off('resize', this.resize);

    if (callback) callback();
};
