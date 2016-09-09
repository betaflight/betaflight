'use strict';

TABS.receiver = {
    rateChartHeight: 117,
    useSuperExpo: false,
    deadband: 0,
    yawDeadband: 0
};

TABS.receiver.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'receiver') {
        GUI.active_tab = 'receiver';
    }

    function get_rc_data() {
        MSP.send_message(MSPCodes.MSP_RC, false, false, get_rc_tuning_data);
    }

    function get_rc_tuning_data() {
        MSP.send_message(MSPCodes.MSP_RC_TUNING, false, false, get_bt_config_data);
    }

    function get_bt_config_data() {
        MSP.send_message(MSPCodes.MSP_BF_CONFIG, false, false, get_rc_map);
    }

    function get_rc_map() {
        MSP.send_message(MSPCodes.MSP_RX_MAP, false, false, load_config);
    }

    // Fetch features so we can check if RX_MSP is enabled:
    function load_config() {
        MSP.send_message(MSPCodes.MSP_BF_CONFIG, false, false, load_rc_configs);
    }

    function load_rc_configs() {
        var next_callback = load_rx_config;
        if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
            MSP.send_message(MSPCodes.MSP_RC_DEADBAND, false, false, next_callback);
        } else {
            next_callback();
        }
    }

    function load_rx_config() {
        var next_callback = load_html;
        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            MSP.send_message(MSPCodes.MSP_RX_CONFIG, false, false, next_callback);
        } else {
            next_callback();
        }
    }

    function load_html() {
        $('#content').load("./tabs/receiver.html", process_html);
    }

    MSP.send_message(MSPCodes.MSP_MISC, false, false, get_rc_data);

    function process_html() {
        // translate to user-selected language
        localize();

        chrome.storage.local.get('rx_refresh_rate', function (result) {
            if (result.rx_refresh_rate) {
                $('select[name="rx_refresh_rate"]').val(result.rx_refresh_rate).change();
            } else {
                $('select[name="rx_refresh_rate"]').change(); // start with default value
            }
        });

        if (semver.lt(CONFIG.apiVersion, "1.15.0")) {
            $('.deadband').hide();
        } else {
            $('.deadband input[name="yaw_deadband"]').val(RC_deadband.yaw_deadband);
            $('.deadband input[name="deadband"]').val(RC_deadband.deadband);

            $('.deadband input[name="deadband"]').change(function () {
                this.deadband = parseInt($(this).val());
            }).change();
            $('.deadband input[name="yaw_deadband"]').change(function () {
                this.yawDeadband = parseInt($(this).val());
            }).change();

        }

        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            $('select[name="rcInterpolation-select"]').val(RX_CONFIG.rcInterpolation);
            $('input[name="rcInterpolationInterval-number"]').val(RX_CONFIG.rcInterpolationInterval);

            $('select[name="rcInterpolation-select"]').change(function () {
                self.updateRcInterpolationParameters();
            }).change();
        } else {
            $('.tab-receiver div.rcInterpolation').hide();
        }

        // generate bars
        var bar_names = [
                chrome.i18n.getMessage('controlAxisRoll'),
                chrome.i18n.getMessage('controlAxisPitch'),
                chrome.i18n.getMessage('controlAxisYaw'),
                chrome.i18n.getMessage('controlAxisThrottle')
            ],
            bar_container = $('.tab-receiver .bars'),
            aux_index = 1;

        for (var i = 0; i < RC.active_channels; i++) {
            var name;
            if (i < bar_names.length) {
                name = bar_names[i];
            } else {
                name = chrome.i18n.getMessage("controlAxisAux" + (aux_index++));
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

        // rssi
        var rssi_channel_e = $('select[name="rssi_channel"]');
        rssi_channel_e.append('<option value="0">Disabled</option>');
        for (var i = 1; i < RC.active_channels + 1; i++) {
            rssi_channel_e.append('<option value="' + i + '">' + i + '</option>');
        }

        $('select[name="rssi_channel"]').val(MISC.rssi_channel);

        var rateHeight = TABS.receiver.rateChartHeight;

        // UI Hooks
        $('a.refresh').click(function () {
	    // Todo: refresh data here
        });

        $('a.update').click(function () {
            if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
               RC_deadband.yaw_deadband = parseInt($('.deadband input[name="yaw_deadband"]').val());
               RC_deadband.deadband = parseInt($('.deadband input[name="deadband"]').val());
            }

            // catch rc map
            var RC_MAP_Letters = ['A', 'E', 'R', 'T', '1', '2', '3', '4'];
            var strBuffer = $('input[name="rcmap"]').val().split('');

            for (var i = 0; i < RC_MAP.length; i++) {
                RC_MAP[i] = strBuffer.indexOf(RC_MAP_Letters[i]);
            }

            // catch rssi aux
            MISC.rssi_channel = parseInt($('select[name="rssi_channel"]').val());

            if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
                RX_CONFIG.rcInterpolation = parseInt($('select[name="rcInterpolation-select"]').val());
                RX_CONFIG.rcInterpolationInterval = parseInt($('input[name="rcInterpolationInterval-number"]').val());
            }

            function save_misc() {
                MSP.send_message(MSPCodes.MSP_SET_MISC, mspHelper.crunch(MSPCodes.MSP_SET_MISC), false, save_rc_configs);
            }

            function save_rc_configs() {
                var next_callback = save_rx_config;
                if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                    MSP.send_message(MSPCodes.MSP_SET_RC_DEADBAND, mspHelper.crunch(MSPCodes.MSP_SET_RC_DEADBAND), false, next_callback);
                } else {
                    next_callback();
                }
            }

            function save_rx_config() {
                var next_callback = save_to_eeprom;
                if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
                    MSP.send_message(MSPCodes.MSP_SET_RX_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_RX_CONFIG), false, next_callback);
                } else {
                    next_callback();
                }
            }

            function save_to_eeprom() {
                MSP.send_message(MSPCodes.MSP_EEPROM_WRITE, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('receiverEepromSaved'));
                });
            }

            MSP.send_message(MSPCodes.MSP_SET_RX_MAP, mspHelper.crunch(MSPCodes.MSP_SET_RX_MAP), false, save_misc);
        });

        $("a.sticks").click(function() {
            var
                windowWidth = 370,
                windowHeight = 510;

            chrome.app.window.create("/tabs/receiver_msp.html", {
                id: "receiver_msp",
                innerBounds: {
                    minWidth: windowWidth, minHeight: windowHeight,
                    width: windowWidth, height: windowHeight,
                    maxWidth: windowWidth, maxHeight: windowHeight
                },
                alwaysOnTop: true
            }, function(createdWindow) {
                // Give the window a callback it can use to send the channels (otherwise it can't see those objects)
                createdWindow.contentWindow.setRawRx = function(channels) {
                    if (CONFIGURATOR.connectionValid && GUI.active_tab != 'cli') {
                        mspHelper.setRawRx(channels);
                        return true;
                    } else {
                        return false;
                    }
                }
            });
        });

        // Only show the MSP control sticks if the MSP Rx feature is enabled
        $(".sticks_btn").toggle(BF_CONFIG.features.isEnabled('RX_MSP'));

        $('select[name="rx_refresh_rate"]').change(function () {
            var plot_update_rate = parseInt($(this).val(), 10);

            // save update rate
            chrome.storage.local.set({'rx_refresh_rate': plot_update_rate});

            function get_rc_data() {
                MSP.send_message(MSPCodes.MSP_RC, false, false, update_ui);
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

        // Setup model for preview
        self.initModelPreview();
        self.renderModel();

        // TODO: Combine two polls together
        GUI.interval_add('receiver_pull_for_model_preview', self.getRecieverData, 33, false);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSPCodes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.receiver.getRecieverData = function () {
    MSP.send_message(MSPCodes.MSP_RC, false, false);
};

TABS.receiver.initModelPreview = function () {
    this.keepRendering = true;
    this.model = new Model($('.model_preview'), $('.model_preview canvas'));

    this.useSuperExpo = false;
    if (semver.gte(CONFIG.flightControllerVersion, '2.8.0')) {
        this.useSuperExpo = BF_CONFIG.features.isEnabled('SUPEREXPO_RATES');
    }

    this.rateCurve = new RateCurve(CONFIG.flightControllerIdentifier !== 'BTFL' || semver.lt(CONFIG.flightControllerVersion, '2.8.0'));

    $(window).on('resize', $.proxy(this.model.resize, this.model));
};

TABS.receiver.renderModel = function () {
    if (this.keepRendering) { requestAnimationFrame(this.renderModel.bind(this)); }

    if (!this.clock) { this.clock = new THREE.Clock(); }

    if (RC.channels[0] && RC.channels[1] && RC.channels[2]) {
        var delta = this.clock.getDelta();

        var roll  = delta * this.rateCurve.rcCommandRawToDegreesPerSecond(RC.channels[0], RC_tuning.roll_rate, RC_tuning.RC_RATE, RC_tuning.RC_EXPO, this.useSuperExpo, this.deadband),
            pitch = delta * this.rateCurve.rcCommandRawToDegreesPerSecond(RC.channels[1], RC_tuning.pitch_rate, RC_tuning.RC_RATE, RC_tuning.RC_EXPO, this.useSuperExpo, this.deadband),
            yaw   = delta * this.rateCurve.rcCommandRawToDegreesPerSecond(RC.channels[2], RC_tuning.yaw_rate, RC_tuning.rcYawRate, RC_tuning.RC_YAW_EXPO, this.useSuperExpo, this.yawDeadband);

        this.model.rotateBy(-degToRad(pitch), -degToRad(yaw), -degToRad(roll));
    }
};


TABS.receiver.cleanup = function (callback) {
    $(window).off('resize', this.resize);
    if (this.model) {
        $(window).off('resize', $.proxy(this.model.resize, this.model));
    }

    this.keepRendering = false;

    if (callback) callback();
};

TABS.receiver.updateRcInterpolationParameters = function () {
    if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
        if ($('select[name="rcInterpolation-select"]').val() === '3') {
            $('.tab-receiver .rcInterpolationInterval').show();
        } else {
            $('.tab-receiver .rcInterpolationInterval').hide();
        }
    }
};
