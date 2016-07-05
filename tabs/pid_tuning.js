'use strict';

TABS.pid_tuning = {
    controllerChanged: false
};

TABS.pid_tuning.initialize = function (callback) {
    var self = this;
    if (GUI.active_tab != 'pid_tuning') {
        GUI.active_tab = 'pid_tuning';
    }

    // requesting MSP_STATUS manually because it contains CONFIG.profile
    MSP.promise(MSP_codes.MSP_STATUS).then(function() {
        if (GUI.canChangePidController) {
            return MSP.promise(MSP_codes.MSP_PID_CONTROLLER);
        }
        return true;
    }).then(function() {
        return MSP.promise(MSP_codes.MSP_PIDNAMES)
    }).then(function() {
        return MSP.promise(MSP_codes.MSP_PID);
    }).then(function() {
        return MSP.promise(MSP_codes.MSP_RC_TUNING);
    }).then(function() {
        return MSP.promise(MSP_codes.MSP_SPECIAL_PARAMETERS);
    }).then(function() {
        return MSP.promise(MSP_codes.MSP_ADVANCED_TUNING);
    }).then(function() {
        return MSP.promise(MSP_codes.MSP_FILTER_CONFIG);
    }).then(function() {
        $('#content').load("./tabs/pid_tuning.html", process_html);
    });

    function pid_and_rc_to_form() {
        // Fill in the data from PIDs array
        var i = 0;
        $('.pid_tuning .ROLL input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[0][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[0][i++]);
                    break;
                case 2:
                    $(this).val(PIDs[0][i++]);
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .PITCH input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[1][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[1][i++]);
                    break;
                case 2:
                    $(this).val(PIDs[1][i++]);
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .YAW input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[2][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[2][i++]);
                    break;
                case 2:
                    $(this).val(PIDs[2][i++]);
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .ALT input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[3][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[3][i++]);
                    break;
                case 2:
                    $(this).val(PIDs[3][i++]);
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .Pos input').each(function () {
            $(this).val(PIDs[4][i++]);
        });

        i = 0;
        $('.pid_tuning .PosR input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[5][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[5][i++]);
                    break;
                case 2:
                    $(this).val(PIDs[5][i++]);
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .NavR input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[6][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[6][i++]);
                    break;
                case 2:
                    $(this).val(PIDs[6][i++]);
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .HORIZON input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[7][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[7][i++]);
                    break;
            }
        });
        $('.pid_tuning .ANGLE input').each(function () {
            switch (i) {
                case 2:
                    $(this).val(PIDs[7][i++]);
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .MAG input').each(function () {
            $(this).val(PIDs[8][i++]);
        });

        i = 0;
        $('.pid_tuning .Vario input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[9][i++]);
                    break;
                case 1:
                    $(this).val(PIDs[9][i++]);
                    break;
                case 2:
                    $(this).val(PIDs[9][i++]);
                    break;
            }
        });

        // Fill in data from RC_tuning object
        $('.pid_tuning input[name="rc_rate"]').val(RC_tuning.RC_RATE.toFixed(2));
        $('.pid_tuning input[name="roll_pitch_rate"]').val(RC_tuning.roll_pitch_rate.toFixed(2));
        $('.pid_tuning input[name="roll_rate"]').val(RC_tuning.roll_rate.toFixed(2));
        $('.pid_tuning input[name="pitch_rate"]').val(RC_tuning.pitch_rate.toFixed(2));
        $('.pid_tuning input[name="yaw_rate"]').val(RC_tuning.yaw_rate.toFixed(2));
        $('.pid_tuning input[name="rc_expo"]').val(RC_tuning.RC_EXPO.toFixed(2));
        $('.pid_tuning input[name="rc_yaw_expo"]').val(RC_tuning.RC_YAW_EXPO.toFixed(2));
        $('.pid_tuning input[name="rc_rate_yaw"]').val(SPECIAL_PARAMETERS.RC_RATE_YAW.toFixed(2));

	$('.throttle input[name="mid"]').val(RC_tuning.throttle_MID.toFixed(2));
	$('.throttle input[name="expo"]').val(RC_tuning.throttle_EXPO.toFixed(2));

        $('.tpa input[name="tpa"]').val(RC_tuning.dynamic_THR_PID.toFixed(2));
        $('.tpa input[name="tpa-breakpoint"]').val(RC_tuning.dynamic_THR_breakpoint);

        if (semver.lt(CONFIG.apiVersion, "1.10.0")) {
            $('.pid_tuning input[name="rc_yaw_expo"]').hide();
            $('.pid_tuning input[name="rc_expo"]').attr("rowspan", "3");
        }

        $('.pid_tuning input[name="gyro"]').val(FILTER_CONFIG.gyro_soft_lpf_hz);
        $('.pid_tuning input[name="dterm"]').val(FILTER_CONFIG.dterm_lpf_hz);
        $('.pid_tuning input[name="yaw"]').val(FILTER_CONFIG.yaw_lpf_hz);

        if (CONFIG.flightControllerIdentifier !== "BTFL" || semver.lt(CONFIG.flightControllerVersion, "2.8.1")) {
            $('.pid_filter').hide();
            $('.pid_tuning input[name="rc_rate_yaw"]').hide();
        }
    }

    function form_to_pid_and_rc() {
        // Catch all the changes and stuff the inside PIDs array
        var i = 0;
        $('table.pid_tuning tr.ROLL input').each(function () {
            PIDs[0][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.PITCH input').each(function () {
            PIDs[1][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.YAW input').each(function () {
            PIDs[2][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.ALT input').each(function () {
            PIDs[3][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.Vario input').each(function () {
            PIDs[9][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.Pos input').each(function () {
            PIDs[4][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.PosR input').each(function () {
            PIDs[5][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.NavR input').each(function () {
            PIDs[6][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.HORIZON input').each(function () {
            PIDs[7][i++] = parseFloat($(this).val());
        });
        $('table.pid_tuning tr.ANGLE input').each(function () {
            PIDs[7][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.MAG input').each(function () {
            PIDs[8][i++] = parseFloat($(this).val());
        });

        // catch RC_tuning changes
        RC_tuning.RC_RATE = parseFloat($('.pid_tuning input[name="rc_rate"]').val());
        RC_tuning.roll_pitch_rate = parseFloat($('.pid_tuning input[name="roll_pitch_rate"]').val());
        RC_tuning.roll_rate = parseFloat($('.pid_tuning input[name="roll_rate"]').val());
        RC_tuning.pitch_rate = parseFloat($('.pid_tuning input[name="pitch_rate"]').val());
        RC_tuning.yaw_rate = parseFloat($('.pid_tuning input[name="yaw_rate"]').val());
        RC_tuning.RC_EXPO = parseFloat($('.pid_tuning input[name="rc_expo"]').val());
        RC_tuning.RC_YAW_EXPO = parseFloat($('.pid_tuning input[name="rc_yaw_expo"]').val());
        SPECIAL_PARAMETERS.RC_RATE_YAW = parseFloat($('.pid_tuning input[name="rc_rate_yaw"]').val());

        RC_tuning.throttle_MID = parseFloat($('.throttle input[name="mid"]').val());
        RC_tuning.throttle_EXPO = parseFloat($('.throttle input[name="expo"]').val())

        RC_tuning.dynamic_THR_PID = parseFloat($('.tpa input[name="tpa"]').val());
        RC_tuning.dynamic_THR_breakpoint = parseInt($('.tpa input[name="tpa-breakpoint"]').val());
        FILTER_CONFIG.gyro_soft_lpf_hz = parseInt($('.pid_tuning input[name="gyro"]').val());
        FILTER_CONFIG.dterm_lpf_hz = parseInt($('.pid_tuning input[name="dterm"]').val());
        FILTER_CONFIG.yaw_lpf_hz = parseInt($('.pid_tuning input[name="yaw"]').val());
    }

    function showAllPids() {
        $('.tab-pid_tuning .pid_tuning').show();
    }

    function hideUnusedPids() {
        $('.tab-pid_tuning .pid_tuning').hide();

        $('#pid_main').show();

        if (CONFIG.flightControllerIdentifier === "BTFL" || semver.ge(CONFIG.flightControllerVersion, "2.9.0")) {
            $('#pid_filter').show();
        }

        if (have_sensor(CONFIG.activeSensors, 'acc')) {
            $('#pid_accel').show();
            $('#pid_level').show();
        }

        var showTitle = false;
        if (have_sensor(CONFIG.activeSensors, 'baro') ||
            have_sensor(CONFIG.activeSensors, 'sonar')) {
            $('#pid_baro').show();
            showTitle = true;
        }
        if (have_sensor(CONFIG.activeSensors, 'mag')) {
            $('#pid_mag').show();
            showTitle = true;
        }
        if (bit_check(BF_CONFIG.features, 7)) {   //This will need to be reworked to remove BF_CONFIG reference eventually
            $('#pid_gps').show();
            showTitle = true;
        }

        if (showTitle) {
            $('#pid_optional').show();
        }
    }

    function drawAxes(curveContext, width, height, scaleHeight) {
        curveContext.strokeStyle = '#000000';
        curveContext.lineWidth = 4;

        // Horizontal
        curveContext.beginPath();
        curveContext.moveTo(0, height / 2);
        curveContext.lineTo(width, height / 2);
        curveContext.stroke();

        // Vertical
        curveContext.beginPath();
        curveContext.moveTo(width / 2, 0);
        curveContext.lineTo(width / 2, height);
        curveContext.stroke();

        if (scaleHeight <= height / 2) {
            curveContext.strokeStyle = '#202020';
            curveContext.lineWidth = 1;

            curveContext.beginPath();
            curveContext.moveTo(0, height / 2 + scaleHeight);
            curveContext.lineTo(width, height / 2 + scaleHeight);
            curveContext.stroke();

            curveContext.beginPath();
            curveContext.moveTo(0, height / 2 - scaleHeight);
            curveContext.lineTo(width, height / 2 - scaleHeight);
            curveContext.stroke();
        }
    }

    function checkInput(element) {
        var value = parseFloat(element.val());
        if (value < parseFloat(element.prop('min'))
            || value > parseFloat(element.prop('max'))) {
            value = undefined;
        }

        return value;
    }

    function process_html() {
        // translate to user-selected language
        localize();

        var showAllButton = $('#showAllPids');
        var showAllMsg = chrome.i18n.getMessage("pidTuningShowAllPids");
        var hideUnusedMsg = chrome.i18n.getMessage("pidTuningHideUnusedPids");

        function updatePidDisplay() {
            if (!TABS.pid_tuning.showAllPids) {
                hideUnusedPids();

                showAllButton.text(showAllMsg);
            } else {
                showAllPids();

                showAllButton.text(hideUnusedMsg);
            }
        }

        updatePidDisplay();

        showAllButton.on('click', function(){
            TABS.pid_tuning.showAllPids = !TABS.pid_tuning.showAllPids;

            updatePidDisplay();
        });

        $('#resetPIDs').on('click', function(){
          MSP.send_message(MSP_codes.MSP_SET_RESET_CURR_PID, false, false, false);
          updateActivatedTab();
        });

        $('.pid_tuning tr').each(function(){
          for(i = 0; i < PID_names.length; i++) {
            if($(this).hasClass(PID_names[i])) {
              $(this).find('td:first').text(PID_names[i]);
            }
          }
        });

        pid_and_rc_to_form();

        var pidController_e = $('select[name="controller"]');

        var pidControllerList;

        if (semver.lt(CONFIG.apiVersion, "1.14.0")) {
            pidControllerList = [
                { name: "MultiWii (Old)"},
                { name: "MultiWii (rewrite)"},
                { name: "LuxFloat"},
                { name: "MultiWii (2.3 - latest)"},
                { name: "MultiWii (2.3 - hybrid)"},
                { name: "Harakiri"}
            ]
        } else {
            pidControllerList = [
                { name: ""},
                { name: "Integer"},
                { name: "Float"},
            ]
        }

        for (var i = 0; i < pidControllerList.length; i++) {
            pidController_e.append('<option value="' + (i) + '">' + pidControllerList[i].name + '</option>');
        }

        var form_e = $('#pid-tuning');

        if (GUI.canChangePidController) {
            pidController_e.val(PID.controller);
        } else {
            GUI.log(chrome.i18n.getMessage('pidTuningUpgradeFirmwareToChangePidController', [CONFIG.apiVersion, CONFIGURATOR.pidControllerChangeMinApiVersion]));

            pidController_e.empty();
            pidController_e.append('<option value="">Unknown</option>');

            pidController_e.prop('disabled', true);
        }

        if (semver.lt(CONFIG.apiVersion, "1.7.0")) {
            $('.tpa .tpa-breakpoint').hide();

            $('.pid_tuning .roll_rate').hide();
            $('.pid_tuning .pitch_rate').hide();
        } else {
            $('.pid_tuning .roll_pitch_rate').hide();
        }

        // Getting the DOM elements for curve display
	var rcRateElement = $('.pid_tuning input[name="rc_rate"]');
        var rcRateYawElement;
        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
            rcRateYawElement = $('.pid_tuning input[name="rc_rate_yaw"]');
        } else {
            rcRateYawElement = rcRateElement;
        }

        var rateRollElement = $('.pid_tuning input[name="roll_rate"]');
        var ratePitchElement = $('.pid_tuning input[name="pitch_rate"]');
        var rateYawElement = $('.pid_tuning input[name="yaw_rate"]');

        var rcExpoElement = $('.pid_tuning input[name="rc_expo"]');
        var rcExpoYawElement = $('.pid_tuning input[name="rc_yaw_expo"]');

        var rcCurveElement = $('.rate_curve canvas').get(0);
        var curveContext = rcCurveElement.getContext("2d");
        rcCurveElement.width = 1000;
        rcCurveElement.height = 1000;

	var maxAngularVelRollElement = $('.rc_curve .maxAngularVelRoll');
	var maxAngularVelPitchElement = $('.rc_curve .maxAngularVelPitch');
	var maxAngularVelYawElement = $('.rc_curve .maxAngularVelYaw');

        var superExpoElement = $('.rc_curve input[name="show_superexpo_rates"]');

	var useLegacyCurve = false;
        if (CONFIG.flightControllerIdentifier !== "BTFL" || semver.lt(CONFIG.flightControllerVersion, "2.8.0")) {
            $('.pid_tuning .new_rates').hide();
            useLegacyCurve = true;
        }

	var rateCurve = new RateCurve(useLegacyCurve);

        // UI Hooks
        // curves
        function redrawRateCurves() {
            setTimeout(function () { // let global validation trigger and adjust the values first
                var curveHeight = rcCurveElement.height;
                var curveWidth = rcCurveElement.width;

		var useSuperExpo = superExpoElement.is(':checked');

                var rateRoll = checkInput(rateRollElement);
                var ratePitch = checkInput(ratePitchElement);
                var rateYaw = checkInput(rateYawElement);
                var rcRate = checkInput(rcRateElement);
                var rcRateYaw = checkInput(rcRateYawElement);
                var rcExpo = checkInput(rcExpoElement);
                var rcExpoYaw = checkInput(rcExpoYawElement);

                var maxAngularVelRoll = rateCurve.getMaxAngularVel(rateRoll, rcRate, rcExpo, useSuperExpo);
		maxAngularVelRollElement.text(maxAngularVelRoll);
                var maxAngularVelPitch = rateCurve.getMaxAngularVel(ratePitch, rcRate, rcExpo, useSuperExpo);
		maxAngularVelPitchElement.text(maxAngularVelPitch);
                var maxAngularVelYaw = rateCurve.getMaxAngularVel(rateYaw, rcRateYaw, rcExpoYaw, useSuperExpo);
		maxAngularVelYawElement.text(maxAngularVelYaw);
		var maxAngularVel = Math.max(maxAngularVelRoll, maxAngularVelPitch, maxAngularVelYaw);

                curveContext.clearRect(0, 0, curveWidth, curveHeight);

                if (!useLegacyCurve) {
                    drawAxes(curveContext, curveWidth, curveHeight, (curveHeight / 2) / maxAngularVel * 360);
                }

                curveContext.lineWidth = 4;

                curveContext.save();
                curveContext.strokeStyle = '#ff0000';
                rateCurve.draw(rateRoll, rcRate, rcExpo, useSuperExpo, maxAngularVel, curveContext);
                curveContext.restore();

                curveContext.save();
                curveContext.translate(0, -4);
                curveContext.strokeStyle = '#00ff00';
                rateCurve.draw(ratePitch, rcRate, rcExpo, useSuperExpo, maxAngularVel, curveContext);
                curveContext.restore();

                curveContext.save();
                curveContext.strokeStyle = '#0000ff';
                curveContext.translate(0, 4);
                rateCurve.draw(rateYaw, rcRateYaw, rcExpoYaw, useSuperExpo, maxAngularVel, curveContext);
                curveContext.restore();
            }, 0);
        };

        $('.pid_tuning').on('input change', redrawRateCurves).trigger('input');
        $('.super_expo_checkbox').on('input change', redrawRateCurves).trigger('input');

        $('.throttle input').on('input change', function () {
            setTimeout(function () { // let global validation trigger and adjust the values first
                var throttleMidE = $('.throttle input[name="mid"]'),
                    throttleExpoE = $('.throttle input[name="expo"]'),
                    mid = parseFloat(throttleMidE.val()),
                    expo = parseFloat(throttleExpoE.val()),
                    throttleCurve = $('.throttle .throttle_curve canvas').get(0),
                    context = throttleCurve.getContext("2d");

                // local validation to deal with input event
                if (mid >= parseFloat(throttleMidE.prop('min')) &&
                    mid <= parseFloat(throttleMidE.prop('max')) &&
                    expo >= parseFloat(throttleExpoE.prop('min')) &&
                    expo <= parseFloat(throttleExpoE.prop('max'))) {
                    // continue
                } else {
                    return;
                }

                var canvasHeight = throttleCurve.height;
                var canvasWidth = throttleCurve.width;

                // math magic by englishman
                var midx = canvasWidth * mid,
                    midxl = midx * 0.5,
                    midxr = (((canvasWidth - midx) * 0.5) + midx),
                    midy = canvasHeight - (midx * (canvasHeight / canvasWidth)),
                    midyl = canvasHeight - ((canvasHeight - midy) * 0.5 *(expo + 1)),
                    midyr = (midy / 2) * (expo + 1);

                // draw
                context.clearRect(0, 0, canvasWidth, canvasHeight);
                context.beginPath();
                context.moveTo(0, canvasHeight);
                context.quadraticCurveTo(midxl, midyl, midx, midy);
                context.moveTo(midx, midy);
                context.quadraticCurveTo(midxr, midyr, canvasWidth, 0);
                context.lineWidth = 2;
                context.strokeStyle = '#ffbb00';
                context.stroke();
            }, 0);
        }).trigger('input');

        $('a.refresh').click(function () {
            GUI.tab_switch_cleanup(function () {
                GUI.log(chrome.i18n.getMessage('pidTuningDataRefreshed'));
                TABS.pid_tuning.initialize();
            });
        });

        form_e.find('input').each(function (k, item) {
            $(item).change(function () {
                pidController_e.prop("disabled", true);
                TABS.pid_tuning.controllerChanged = false;
            })
        });

        pidController_e.change(function () {
            if (PID.controller != pidController_e.val()) {
                form_e.find('input').each(function (k, item) {
                    $(item).prop('disabled', true);
                    TABS.pid_tuning.controllerChanged = true;
                });
            }
        });

        $('.delta select').val(ADVANCED_TUNING.deltaMethod).change(function() {
            ADVANCED_TUNING.deltaMethod = $(this).val();
        });

        if (CONFIG.flightControllerIdentifier == "BTFL" && semver.lt(CONFIG.flightControllerVersion, "2.8.2")) {
            $('.delta').hide();
            $('.note').hide();
        }

        // update == save.
        $('a.update').click(function () {
            form_to_pid_and_rc();
            if (GUI.canChangePidController && TABS.pid_tuning.controllerChanged) {
                PID.controller = pidController_e.val();
                MSP.send_message(MSP_codes.MSP_SET_PID_CONTROLLER, MSP.crunch(MSP_codes.MSP_SET_PID_CONTROLLER), false, function () {
                    MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('pidTuningEepromSaved'));
                    });
                    TABS.pid_tuning.initialize();
                });
            } else {
                if (TABS.pid_tuning.controllerChanged) { return; }
                MSP.promise(MSP_codes.MSP_SET_PID, MSP.crunch(MSP_codes.MSP_SET_PID)).then(function() {
                    if (TABS.pid_tuning.controllerChanged) { Promise.reject('pid controller changed'); }
                    if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
                        return MSP.promise(MSP_codes.MSP_SET_SPECIAL_PARAMETERS, MSP.crunch(MSP_codes.MSP_SET_SPECIAL_PARAMETERS));
                    }
                }).then(function() {
                    if (TABS.pid_tuning.controllerChanged) { Promise.reject('pid controller changed'); }
                    return MSP.promise(MSP_codes.MSP_SET_ADVANCED_TUNING, MSP.crunch(MSP_codes.MSP_SET_ADVANCED_TUNING));
                }).then(function() {
                    if (TABS.pid_tuning.controllerChanged) { Promise.reject('pid controller changed'); }
                    if (CONFIG.flightControllerIdentifier == "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
                        return MSP.promise(MSP_codes.MSP_SET_FILTER_CONFIG, MSP.crunch(MSP_codes.MSP_SET_FILTER_CONFIG));
                    }
                }).then(function() {
                    return MSP.promise(MSP_codes.MSP_SET_RC_TUNING, MSP.crunch(MSP_codes.MSP_SET_RC_TUNING));
                }).then(function() {
                    return MSP.promise(MSP_codes.MSP_EEPROM_WRITE);
                }).then(function() {
                    GUI.log(chrome.i18n.getMessage('pidTuningEepromSaved'));
                });
            }
        });

        // Setup model for rates preview
        self.initRatesPreview();
        self.renderModel();

        // enable RC data pulling for rates preview
        GUI.interval_add('receiver_pull', self.getRecieverData, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.pid_tuning.getRecieverData = function () {
    MSP.send_message(MSP_codes.MSP_RC, false, false);
};

TABS.pid_tuning.initRatesPreview = function () {
    this.keepRendering = true;
    this.model = new Model($('.rates_preview'), $('.rates_preview canvas'));

    var scale = d3.scale.linear().domain([900, 2100]);

    this.rollScale = scale.range([Math.PI * 2, -Math.PI * 2]);
    this.pitchScale = scale.range([Math.PI * 2, -Math.PI * 2]);
    this.yawScale = scale.range([Math.PI * 2, -Math.PI * 2]);

    $(window).on('resize', $.proxy(this.model.resize, this.model));
};

TABS.pid_tuning.renderModel = function () {
    if (this.keepRendering) { requestAnimationFrame(this.renderModel.bind(this)); }

    if (!this.clock) { this.clock = new THREE.Clock(); }

    if (RC.channels[0] && RC.channels[1] && RC.channels[2]) {
        var delta = this.clock.getDelta(),
            roll  = delta * this.rollScale(RC.channels[0]),
            pitch = delta * this.pitchScale(RC.channels[1]),
            yaw   = delta * this.yawScale(RC.channels[2]);

        this.model.rotateBy(pitch, yaw, roll);
    }
};

TABS.pid_tuning.cleanup = function (callback) {
    $(window).off('resize', $.proxy(this.model.resize, this.model));

    this.keepRendering = false;

    if (callback) callback();
};
