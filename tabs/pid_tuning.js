'use strict';

TABS.pid_tuning = {
    RATE_PROFILE_MASK: 128,
    showAllPids: false,
    updating: true,
    dirty: false,
    currentProfile: null,
    currentRateProfile: null
};

TABS.pid_tuning.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab !== 'pid_tuning') {
        GUI.active_tab = 'pid_tuning';
    }

    // requesting MSP_STATUS manually because it contains CONFIG.profile
    MSP.promise(MSPCodes.MSP_STATUS).then(function() {
        if (semver.gte(CONFIG.apiVersion, CONFIGURATOR.pidControllerChangeMinApiVersion)) {
            return MSP.promise(MSPCodes.MSP_PID_CONTROLLER);
        }
    }).then(function() {
        return MSP.promise(MSPCodes.MSP_PIDNAMES)
    }).then(function() {
        return MSP.promise(MSPCodes.MSP_PID);
    }).then(function () {
        if (semver.gte(CONFIG.flightControllerVersion, "2.9.0") && semver.lt(CONFIG.flightControllerVersion, "2.9.1")) {
          return MSP.promise(MSPCodes.MSP_SPECIAL_PARAMETERS);
        }
    }).then(function() {
        if (semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
          return MSP.promise(MSPCodes.MSP_PID_ADVANCED);
        }
    }).then(function() {
        return MSP.promise(MSPCodes.MSP_RC_TUNING);
    }).then(function() {
        return MSP.promise(MSPCodes.MSP_FILTER_CONFIG);
    }).then(function() {
        var promise = true;
        if (CONFIG.flightControllerIdentifier === "BTFL" && semver.gte(CONFIG.flightControllerVersion, "2.8.0")) {
            promise = MSP.promise(MSPCodes.MSP_BF_CONFIG);
        }

        return promise;
    }).then(function() {
        var promise = true;
        if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
            promise = MSP.promise(MSPCodes.MSP_RC_DEADBAND);
        }

        return promise;
    }).then(function() {
        $('#content').load("./tabs/pid_tuning.html", process_html);
    });

    function pid_and_rc_to_form() {
        self.setProfile();
        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            self.setRateProfile();
        }

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
            }
        });
        $('.pid_tuning .YAW_JUMP_PREVENTION input').each(function () {
            switch (i) {
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
        $('.pid_tuning .ANGLE input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[7][i++]);
                    break;
            }
        });
        $('.pid_tuning .HORIZON input').each(function () {
            switch (i) {
                case 1:
                    $(this).val(PIDs[7][i++]);
                    break;
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

        $('.throttle input[name="mid"]').val(RC_tuning.throttle_MID.toFixed(2));
        $('.throttle input[name="expo"]').val(RC_tuning.throttle_EXPO.toFixed(2));

        $('.tpa input[name="tpa"]').val(RC_tuning.dynamic_THR_PID.toFixed(2));
        $('.tpa input[name="tpa-breakpoint"]').val(RC_tuning.dynamic_THR_breakpoint);

        if (semver.lt(CONFIG.apiVersion, "1.10.0")) {
            $('.pid_tuning input[name="rc_yaw_expo"]').hide();
            $('.pid_tuning input[name="rc_expo"]').attr("rowspan", "3");
        }

        if (semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
            $('input[id="vbatpidcompensation"]').prop('checked', ADVANCED_TUNING.vbatPidCompensation !== 0);
        }

        if (semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
            $('#pid-tuning .delta select').val(ADVANCED_TUNING.deltaMethod);
        }

        if (semver.gte(CONFIG.flightControllerVersion, '2.9.0')) {
            $('.pid_tuning input[name="rc_rate_yaw"]').val(RC_tuning.rcYawRate.toFixed(2));
            $('.pid_filter input[name="gyroLowpassFrequency"]').val(FILTER_CONFIG.gyro_soft_lpf_hz);
            $('.pid_filter input[name="dtermLowpassFrequency"]').val(FILTER_CONFIG.dterm_lpf_hz);
            $('.pid_filter input[name="yawLowpassFrequency"]').val(FILTER_CONFIG.yaw_lpf_hz);
        } else {
            $('.tab-pid_tuning .subtab-filter').hide();
            $('.tab-pid_tuning .tab_container').hide();
            $('.pid_tuning input[name="rc_rate_yaw"]').hide();
        }

        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")
            || semver.gte(CONFIG.flightControllerVersion, "2.8.0") && BF_CONFIG.features.isEnabled('SUPEREXPO_RATES')) {
            $('#pid-tuning .rate').text(chrome.i18n.getMessage("pidTuningSuperRate"));
        } else {
            $('#pid-tuning .rate').text(chrome.i18n.getMessage("pidTuningRate"));
        }

        if (semver.gte(CONFIG.flightControllerVersion, '3.0.0')) {
            $('.pid_filter input[name="gyroNotchFrequency"]').val(FILTER_CONFIG.gyro_soft_notch_hz);
            $('.pid_filter input[name="gyroNotchCutoff"]').val(FILTER_CONFIG.gyro_soft_notch_cutoff);
            $('.pid_filter input[name="dTermNotchFrequency"]').val(FILTER_CONFIG.dterm_notch_hz);
            $('.pid_filter input[name="dTermNotchCutoff"]').val(FILTER_CONFIG.dterm_notch_cutoff);

            $('input[name="ptermSetpoint-number"]').val(ADVANCED_TUNING.ptermSetpointWeight / 100);
            $('input[name="ptermSetpoint-range"]').val(ADVANCED_TUNING.ptermSetpointWeight / 100);

            $('input[name="dtermSetpoint-number"]').val(ADVANCED_TUNING.dtermSetpointWeight / 100);
            $('input[name="dtermSetpoint-range"]').val(ADVANCED_TUNING.dtermSetpointWeight / 100);
        } else {
            $('.pid_filter .newFilter').hide();
        }
    }

    function form_to_pid_and_rc() {
        // Fill in the data from PIDs array
        // Catch all the changes and stuff the inside PIDs array
        var i = 0;
        $('table.pid_tuning tr.ROLL .pid_data input').each(function () {
            PIDs[0][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.PITCH .pid_data input').each(function () {
            PIDs[1][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.YAW .pid_data input').each(function () {
            PIDs[2][i++] = parseFloat($(this).val());
        });
        $('table.pid_tuning tr.YAW_JUMP_PREVENTION .pid_data input').each(function () {
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
        $('table.pid_tuning tr.ANGLE input').each(function () {
            PIDs[7][i++] = parseFloat($(this).val());
        });
        $('table.pid_tuning tr.HORIZON input').each(function () {
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
        RC_tuning.rcYawRate = parseFloat($('.pid_tuning input[name="rc_rate_yaw"]').val());

        RC_tuning.throttle_MID = parseFloat($('.throttle input[name="mid"]').val());
        RC_tuning.throttle_EXPO = parseFloat($('.throttle input[name="expo"]').val())

        RC_tuning.dynamic_THR_PID = parseFloat($('.tpa input[name="tpa"]').val());
        RC_tuning.dynamic_THR_breakpoint = parseInt($('.tpa input[name="tpa-breakpoint"]').val());
        FILTER_CONFIG.gyro_soft_lpf_hz = parseInt($('.pid_filter input[name="gyroLowpassFrequency"]').val());
        FILTER_CONFIG.dterm_lpf_hz = parseInt($('.pid_filter input[name="dtermLowpassFrequency"]').val());
        FILTER_CONFIG.yaw_lpf_hz = parseInt($('.pid_filter input[name="yawLowpassFrequency"]').val());

        if (semver.gte(CONFIG.flightControllerVersion, "2.8.0") && !semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            BF_CONFIG.features.updateData($('input[name="SUPEREXPO_RATES"]'));
        }

        if (semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
            ADVANCED_TUNING.vbatPidCompensation = $('input[id="vbatpidcompensation"]').is(':checked') ? 1 : 0;
        }

        if (semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
            ADVANCED_TUNING.deltaMethod = $('#pid-tuning .delta select').val();
        }

        if (semver.gte(CONFIG.flightControllerVersion, '3.0.0')) {
            ADVANCED_TUNING.ptermSetpointWeight = parseInt($('input[name="ptermSetpoint-number"]').val() * 100);
            ADVANCED_TUNING.dtermSetpointWeight = parseInt($('input[name="dtermSetpoint-number"]').val() * 100);

            FILTER_CONFIG.gyro_soft_notch_hz = parseInt($('.pid_filter input[name="gyroNotchFrequency"]').val());
            FILTER_CONFIG.gyro_soft_notch_cutoff = parseInt($('.pid_filter input[name="gyroNotchCutoff"]').val());
            FILTER_CONFIG.dterm_notch_hz = parseInt($('.pid_filter input[name="dTermNotchFrequency"]').val());
            FILTER_CONFIG.dterm_notch_cutoff = parseInt($('.pid_filter input[name="dTermNotchCutoff"]').val());
        }

    }

    function showAllPids() {
        $('.tab-pid_tuning .pid_tuning').show();
    }

    function hideUnusedPids() {
        $('.tab-pid_tuning .pid_tuning').hide();

        $('#pid_main').show();

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
        if (BF_CONFIG.features.isEnabled('GPS')) {
            $('#pid_gps').show();
            showTitle = true;
        }

        if (showTitle) {
            $('#pid_optional').show();
        }
    }

    function drawAxes(curveContext, width, height) {
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

    }


    function checkInput(element) {
        var value = parseFloat(element.val());
        if (value < parseFloat(element.prop('min'))
            || value > parseFloat(element.prop('max'))) {
            value = undefined;
        }

        return value;
    }

    var useLegacyCurve = false;
    if (!semver.gte(CONFIG.flightControllerVersion, "2.8.0")) {
        useLegacyCurve = true;
    }

    self.rateCurve = new RateCurve(useLegacyCurve);

    function printMaxAngularVel(rate, rcRate, rcExpo, useSuperExpo, deadband, maxAngularVelElement) {
        var maxAngularVel = self.rateCurve.getMaxAngularVel(rate, rcRate, rcExpo, useSuperExpo, deadband).toFixed(0);
        maxAngularVelElement.text(maxAngularVel);

        return maxAngularVel;
    }

    function drawCurve(rate, rcRate, rcExpo, useSuperExpo, maxAngularVel, deadband, colour, yOffset, context) {
        context.save();
        context.strokeStyle = colour;
        context.translate(0, yOffset);
        self.rateCurve.draw(rate, rcRate, rcExpo, useSuperExpo, maxAngularVel, deadband, context);
        context.restore();
    }

    function process_html() {
        if (semver.gte(CONFIG.flightControllerVersion, "2.8.0") && !semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            BF_CONFIG.features.generateElements($('.tab-pid_tuning .features'));
        } else {
            $('.tab-pid_tuning .pidTuningFeatures').hide();
        }

        // translate to user-selected language
        localize();

        // Local cache of current rates
        self.currentRates = {
            roll_rate:   RC_tuning.roll_rate,
            pitch_rate:  RC_tuning.pitch_rate,
            yaw_rate:    RC_tuning.yaw_rate,
            rc_rate:     RC_tuning.RC_RATE,
            rc_rate_yaw: RC_tuning.rcYawRate,
            rc_expo:     RC_tuning.RC_EXPO,
            rc_yaw_expo: RC_tuning.RC_YAW_EXPO,
            superexpo:   BF_CONFIG.features.isEnabled('SUPEREXPO_RATES'),
            deadband: RC_deadband.deadband,
            yawDeadband: RC_deadband.yaw_deadband
        };

        if (semver.lt(CONFIG.apiVersion, "1.7.0")) {
            self.currentRates.roll_rate = RC_tuning.roll_pitch_rate;
            self.currentRates.pitch_rate = RC_tuning.roll_pitch_rate;
        }

        if (semver.lt(CONFIG.flightControllerVersion, "2.8.1")) {
            self.currentRates.rc_rate_yaw = self.currentRates.rc_rate;
        }

        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            self.currentRates.superexpo = true;
        }

        $('.tab-pid_tuning .tab_container .pid').on('click', function () {
            $('.tab-pid_tuning .subtab-pid').show();
            $('.tab-pid_tuning .subtab-filter').hide();

            $('.tab-pid_tuning .tab_container td').removeClass('active');
            $(this).addClass('active');
        });

        $('.tab-pid_tuning .tab_container .filter').on('click', function () {
            $('.tab-pid_tuning .subtab-filter').show();
            $('.tab-pid_tuning .subtab-pid').hide();

            $('.tab-pid_tuning .tab_container td').removeClass('active');
            $(this).addClass('active');
        });

        var showAllButton = $('#showAllPids');

        function updatePidDisplay() {
            if (!self.showAllPids) {
                hideUnusedPids();

                showAllButton.text(chrome.i18n.getMessage("pidTuningShowAllPids"));
            } else {
                showAllPids();

                showAllButton.text(chrome.i18n.getMessage("pidTuningHideUnusedPids"));
            }
        }

        updatePidDisplay();

        showAllButton.on('click', function(){
            self.showAllPids = !self.showAllPids;

            updatePidDisplay();
        });

        $('#resetProfile').on('click', function(){
            self.updating = true;
            MSP.promise(MSPCodes.MSP_SET_RESET_CURR_PID).then(function () {
                self.refresh(function () {
                    self.updating = false;

                    GUI.log(chrome.i18n.getMessage('pidTuningProfileReset'));
                });
            });
        });

        $('.tab-pid_tuning select[name="profile"]').change(function () {
            self.currentProfile = parseInt($(this).val());
            self.updating = true;
            MSP.promise(MSPCodes.MSP_SELECT_SETTING, [self.currentProfile]).then(function () {
                self.refresh(function () {
                    self.updating = false;

                    GUI.log(chrome.i18n.getMessage('pidTuningLoadedProfile', [self.currentProfile + 1]));
                });
            });
        });

        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            $('.tab-pid_tuning select[name="rate_profile"]').change(function () {
                self.currentRateProfile = parseInt($(this).val());
                self.updating = true;
                MSP.promise(MSPCodes.MSP_SELECT_SETTING, [self.currentRateProfile + self.RATE_PROFILE_MASK]).then(function () {
                    self.refresh(function () {
                        self.updating = false;

                        GUI.log(chrome.i18n.getMessage('pidTuningLoadedRateProfile', [self.currentRateProfile + 1]));
                    });
                });
            });

            var ptermNumberElement = $('input[name="ptermSetpoint-number"]');
            var ptermRangeElement = $('input[name="ptermSetpoint-range"]');
            ptermNumberElement.change(function () {
                ptermRangeElement.val($(this).val());
            });
            ptermRangeElement.change(function () {
                ptermNumberElement.val($(this).val());
            });

            var dtermNumberElement = $('input[name="dtermSetpoint-number"]');
            var dtermRangeElement = $('input[name="dtermSetpoint-range"]');
            dtermNumberElement.change(function () {
                dtermRangeElement.val($(this).val());
            });
            dtermRangeElement.change(function () {
                dtermNumberElement.val($(this).val());
            });
        } else {
            $('.tab-pid_tuning .rate_profile').hide();

            $('#pid-tuning .ptermSetpoint').hide();
            $('#pid-tuning .dtermSetpoint').hide();
        }
        
        if (!semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
            $('#pid-tuning .delta').hide();
            $('.tab-pid_tuning .note').hide();
		}

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
        } else if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            pidControllerList = [
                { name: "Legacy"},
                { name: "Betaflight"},
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

        if (semver.gte(CONFIG.apiVersion, CONFIGURATOR.pidControllerChangeMinApiVersion)) {
            pidController_e.val(PID.controller);

			self.updatePidControllerParameters();
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

        if (useLegacyCurve) {
            $('.new_rates').hide();
        }

        // Getting the DOM elements for curve display
        var rcCurveElement              = $('.rate_curve canvas#rate_curve_layer0').get(0),
            curveContext                = rcCurveElement.getContext("2d"),
            updateNeeded                = true,
            maxAngularVel;

        // make these variables global scope so that they can be accessed by the updateRates function.
        self.maxAngularVelRollElement    = $('.pid_tuning .maxAngularVelRoll');
        self.maxAngularVelPitchElement   = $('.pid_tuning .maxAngularVelPitch');
        self.maxAngularVelYawElement     = $('.pid_tuning .maxAngularVelYaw');

        rcCurveElement.width = 1000;
        rcCurveElement.height = 1000;

        function updateRates (event) {
            setTimeout(function () { // let global validation trigger and adjust the values first
                if(event) { // if an event is passed, then use it
                    var targetElement = $(event.target),
                        targetValue = checkInput(targetElement);

                    if (self.currentRates.hasOwnProperty(targetElement.attr('name')) && targetValue !== undefined) {
                        self.currentRates[targetElement.attr('name')] = targetValue;

                        updateNeeded = true;
                    }

                    if (targetElement.attr('name') === 'rc_rate' && semver.lt(CONFIG.flightControllerVersion, "2.8.1")) {
                        self.currentRates.rc_rate_yaw = targetValue;
                    }

                    if (targetElement.attr('name') === 'roll_pitch_rate' && semver.lt(CONFIG.apiVersion, "1.7.0")) {
                        self.currentRates.roll_rate = targetValue;
                        self.currentRates.pitch_rate = targetValue;

                        updateNeeded = true;
                    }

                    if (targetElement.attr('name') === 'SUPEREXPO_RATES') {
                        self.currentRates.superexpo = targetElement.is(':checked');

                        updateNeeded = true;
                    }
                } else { // no event was passed, just force a graph update
                    updateNeeded = true;
                }
                if (updateNeeded) {
                    var curveHeight = rcCurveElement.height;
                    var curveWidth = rcCurveElement.width;
                    var lineScale = curveContext.canvas.width / curveContext.canvas.clientWidth;

                    curveContext.clearRect(0, 0, curveWidth, curveHeight);

                    if (!useLegacyCurve) {
                        maxAngularVel = Math.max(
                            printMaxAngularVel(self.currentRates.roll_rate, self.currentRates.rc_rate, self.currentRates.rc_expo, self.currentRates.superexpo, self.currentRates.deadband, self.maxAngularVelRollElement),
                            printMaxAngularVel(self.currentRates.pitch_rate, self.currentRates.rc_rate, self.currentRates.rc_expo, self.currentRates.superexpo, self.currentRates.deadband, self.maxAngularVelPitchElement),
                            printMaxAngularVel(self.currentRates.yaw_rate, self.currentRates.rc_rate_yaw, self.currentRates.rc_yaw_expo, self.currentRates.superexpo, self.currentRates.yawDeadband, self.maxAngularVelYawElement));

                        // make maxAngularVel multiple of 200deg/s so that the auto-scale doesn't keep changing for small changes of the maximum curve
                        maxAngularVel = self.rateCurve.setMaxAngularVel(maxAngularVel);

                        drawAxes(curveContext, curveWidth, curveHeight);

                    } else {
                        maxAngularVel = 0;
                    }

                    curveContext.lineWidth = 2 * lineScale;
                    drawCurve(self.currentRates.roll_rate, self.currentRates.rc_rate, self.currentRates.rc_expo, self.currentRates.superexpo, self.currentRates.deadband, maxAngularVel, '#ff0000', 0, curveContext);
                    drawCurve(self.currentRates.pitch_rate, self.currentRates.rc_rate, self.currentRates.rc_expo, self.currentRates.superexpo, self.currentRates.deadband, maxAngularVel, '#00ff00', -4, curveContext);
                    drawCurve(self.currentRates.yaw_rate, self.currentRates.rc_rate_yaw, self.currentRates.rc_yaw_expo, self.currentRates.superexpo, self.currentRates.yawDeadband, maxAngularVel, '#0000ff', 4, curveContext);

                    self.updateRatesLabels();

                    updateNeeded = false;
                }
            }, 0);
        };

        // UI Hooks
        // curves
        $('input.feature').on('input change', updateRates);
        $('.pid_tuning').on('input change', updateRates).trigger('input');

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
            self.refresh(function () {
                GUI.log(chrome.i18n.getMessage('pidTuningDataRefreshed'));
            });
        });

        $('#pid-tuning').find('input').each(function (k, item) {
            if ($(item).attr('class') !== "feature toggle"
                && $(item).attr('class') !== "nonProfile") {
                $(item).change(function () {
                    self.setDirty(true);
                });
            }
        });

        if (semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
            $('#pid-tuning .delta select').change(function() {
                self.setDirty(true);
            });
        }

        pidController_e.change(function () {
            self.setDirty(true);

			self.updatePidControllerParameters();
        });

        // update == save.
        $('a.update').click(function () {
            form_to_pid_and_rc();

            self.updating = true;
            Promise.resolve(true)
            .then(function () {
                var promise;
                if (semver.gte(CONFIG.apiVersion, CONFIGURATOR.pidControllerChangeMinApiVersion)) {
                    PID.controller = pidController_e.val();
                    promise = MSP.promise(MSPCodes.MSP_SET_PID_CONTROLLER, mspHelper.crunch(MSPCodes.MSP_SET_PID_CONTROLLER));
                }
                return promise;
            }).then(function () {
                return MSP.promise(MSPCodes.MSP_SET_PID, mspHelper.crunch(MSPCodes.MSP_SET_PID));
            }).then(function () {
                if (semver.gte(CONFIG.flightControllerVersion, "2.9.0") && semver.lt(CONFIG.flightControllerVersion, "3.0.0")) {
                  return MSP.promise(MSPCodes.MSP_SET_SPECIAL_PARAMETERS, mspHelper.crunch(MSPCodes.MSP_SET_SPECIAL_PARAMETERS));
                }
            }).then(function () {
                if (semver.gte(CONFIG.flightControllerVersion, "2.8.2")) {
                  return MSP.promise(MSPCodes.MSP_SET_PID_ADVANCED, mspHelper.crunch(MSPCodes.MSP_SET_PID_ADVANCED));
                }
            }).then(function () {
                if (semver.gte(CONFIG.flightControllerVersion, "2.8.1")) {
                    return MSP.promise(MSPCodes.MSP_SET_FILTER_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_FILTER_CONFIG));
                }
            }).then(function () {
                return MSP.promise(MSPCodes.MSP_SET_RC_TUNING, mspHelper.crunch(MSPCodes.MSP_SET_RC_TUNING));
            }).then(function () {
                if (semver.gte(CONFIG.flightControllerVersion, "2.8.0")) {
                    return MSP.promise(MSPCodes.MSP_SET_BF_CONFIG, mspHelper.crunch(MSPCodes.MSP_SET_BF_CONFIG));
                }
            }).then(function () {
                return MSP.promise(MSPCodes.MSP_EEPROM_WRITE);
            }).then(function () {
                self.updating = false;
                self.setDirty(false);

                GUI.log(chrome.i18n.getMessage('pidTuningEepromSaved'));
            });
        });

        // Setup model for rates preview
        self.initRatesPreview();
        self.renderModel();

        self.updating = false;

        // enable RC data pulling for rates preview
        GUI.interval_add('receiver_pull', self.getRecieverData, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSPCodes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.pid_tuning.getRecieverData = function () {
    MSP.send_message(MSPCodes.MSP_RC, false, false);
};

TABS.pid_tuning.initRatesPreview = function () {
    this.keepRendering = true;
    this.model = new Model($('.rates_preview'), $('.rates_preview canvas'));

    $(window).on('resize', $.proxy(this.model.resize, this.model));
    $(window).on('resize', $.proxy(this.updateRatesLabels, this));
};

TABS.pid_tuning.renderModel = function () {
    if (this.keepRendering) { requestAnimationFrame(this.renderModel.bind(this)); }

    if (!this.clock) { this.clock = new THREE.Clock(); }

    if (RC.channels[0] && RC.channels[1] && RC.channels[2]) {
        var delta = this.clock.getDelta();

        var roll  = delta * this.rateCurve.rcCommandRawToDegreesPerSecond(RC.channels[0], this.currentRates.roll_rate,  this.currentRates.rc_rate,     this.currentRates.rc_expo,     this.currentRates.superexpo, this.currentRates.deadband),
            pitch = delta * this.rateCurve.rcCommandRawToDegreesPerSecond(RC.channels[1], this.currentRates.pitch_rate, this.currentRates.rc_rate,     this.currentRates.rc_expo,     this.currentRates.superexpo, this.currentRates.deadband),
            yaw   = delta * this.rateCurve.rcCommandRawToDegreesPerSecond(RC.channels[2], this.currentRates.yaw_rate,   this.currentRates.rc_rate_yaw, this.currentRates.rc_yaw_expo, this.currentRates.superexpo, this.currentRates.yawDeadband);

        this.model.rotateBy(-degToRad(pitch), -degToRad(yaw), -degToRad(roll));

        if (this.checkRC()) this.updateRatesLabels(); // has the RC data changed ?

    }
};

TABS.pid_tuning.cleanup = function (callback) {
    var self = this;

    if (self.model) {
        $(window).off('resize', $.proxy(self.model.resize, self.model));
    }

    $(window).off('resize', $.proxy(this.updateRatesLabels, this));


    self.keepRendering = false;

    if (callback) callback();
};

TABS.pid_tuning.refresh = function (callback) {
    var self = this;

    GUI.tab_switch_cleanup(function () {
        self.initialize();

        self.setDirty(false);

        if (callback) {
            callback();
        }
    });
}

TABS.pid_tuning.setProfile = function () {
    var self = this;

    self.currentProfile = CONFIG.profile;
    $('.tab-pid_tuning select[name="profile"]').val(self.currentProfile);
}

TABS.pid_tuning.setRateProfile = function () {
    var self = this;

    self.currentRateProfile = CONFIG.rateProfile;
    $('.tab-pid_tuning select[name="rate_profile"]').val(self.currentRateProfile);
}

TABS.pid_tuning.setDirty = function (isDirty) {
    var self = this;

    self.dirty = isDirty;
    $('.tab-pid_tuning select[name="profile"]').prop('disabled', isDirty);
    if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
        $('.tab-pid_tuning select[name="rate_profile"]').prop('disabled', isDirty);
    }
}

TABS.pid_tuning.checkUpdateProfile = function (updateRateProfile) {
    var self = this;

    if (GUI.active_tab === 'pid_tuning') {
        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")
            && CONFIG.numProfiles === 2) {
            $('.tab-pid_tuning select[name="profile"] .profile3').hide();
        }

        if (!self.updating && !self.dirty) {
            var changedProfile = false;
            if (self.currentProfile !== CONFIG.profile) {
                self.setProfile();

                changedProfile = true;
            }

            var changedRateProfile = false;
            if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")
                && updateRateProfile
                && self.currentRateProfile !== CONFIG.rateProfile) {
                self.setRateProfile();

                changedRateProfile = true;
            }

            if (changedProfile || changedRateProfile) {
                self.refresh(function () {
                    if (changedProfile) {
                        GUI.log(chrome.i18n.getMessage('pidTuningReceivedProfile', [CONFIG.profile + 1]));
                    }

                    if (changedRateProfile) {
                        GUI.log(chrome.i18n.getMessage('pidTuningReceivedRateProfile', [CONFIG.rateProfile + 1]));
                    }
                });
            }
        }
    }
}

TABS.pid_tuning.checkRC = function() {
    // Function monitors for change in the primary axes rc received data and returns true if a change is detected.

    if (!this.oldRC) { this.oldRC = [RC.channels[0], RC.channels[1], RC.channels[2]]; }

    // Monitor RC.channels and detect change of value;
    var rateCurveUpdateRequired = false;
    for(var i=0; i<this.oldRC.length; i++) { // has the value changed ?
        if(this.oldRC[i] != RC.channels[i]) {
            this.oldRC[i] = RC.channels[i];
            rateCurveUpdateRequired = true;     // yes, then an update of the values displayed on the rate curve graph is required
        }
    }
    return rateCurveUpdateRequired;
};

TABS.pid_tuning.updatePidControllerParameters = function () {
    if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
        if ($('.tab-pid_tuning select[name="controller"]').val() === '0') {
            $('.pid_tuning .YAW_JUMP_PREVENTION').show();

            $('#pid-tuning .delta').show();

            $('#pid-tuning .ptermSetpoint').hide();
            $('#pid-tuning .dtermSetpoint').hide();
        } else {
            $('.pid_tuning .YAW_JUMP_PREVENTION').hide();

            $('#pid-tuning .ptermSetpoint').show();
            $('#pid-tuning .dtermSetpoint').show();

            $('#pid-tuning .delta').hide();
        }
    }
}

TABS.pid_tuning.updateRatesLabels = function() {
    var self = this;
    if (!self.rateCurve.useLegacyCurve && self.rateCurve.maxAngularVel) {

        var drawAxisLabel = function(context, axisLabel, x, y, align, color) {

            context.fillStyle = color || '#000000' ;
            context.textAlign = align || 'center';
            context.fillText(axisLabel, x, y);
        }
        var drawBalloonLabel = function(context, axisLabel, x, y, align, colors, dirty) {

            /**
             * curveContext is the canvas to draw on
             * axisLabel is the string to display in the center of the balloon
             * x, y are the coordinates of the point of the balloon
             * align is whether the balloon appears to the left (align 'right') or right (align left) of the x,y coordinates
             * colors is an object defining color, border and text are the fill color, border color and text color of the balloon
             */

            const DEFAULT_OFFSET        = 125; // in canvas scale; this is the horizontal length of the pointer
            const DEFAULT_RADIUS        = 10; // in canvas scale, this is the radius around the balloon
            const DEFAULT_MARGIN        = 5;  // in canvas scale, this is the margin around the balloon when it overlaps

            const fontSize = parseInt(context.font);

            // calculate the width and height required for the balloon
            const width = (context.measureText(axisLabel).width * 1.2);
            const height = fontSize * 1.5; // the balloon is bigger than the text height
            const pointerY = y; // always point to the required Y
            // coordinate, even if we move the balloon itself to keep it on the canvas

            // setup balloon background
            context.fillStyle   = colors.color   || '#ffffff' ;
            context.strokeStyle = colors.border  || '#000000' ;

            // correct x position to account for window scaling
            x *= context.canvas.clientWidth/context.canvas.clientHeight;

            // adjust the coordinates for determine where the balloon background should be drawn
            x += ((align=='right')?-(width + DEFAULT_OFFSET):0) + ((align=='left')?DEFAULT_OFFSET:0);
            y -= (height/2); if(y<0) y=0; else if(y>context.height) y=context.height; // prevent balloon from going out of canvas

            // check that the balloon does not already overlap
            for(var i=0; i<dirty.length; i++) {
                if((x>=dirty[i].left && x<=dirty[i].right) || (x+width>=dirty[i].left && x+width<=dirty[i].right)) { // does it overlap horizontally
                    if((y>=dirty[i].top && y<=dirty[i].bottom) || (y+height>=dirty[i].top && y+height<=dirty[i].bottom )) { // this overlaps another balloon
                        // snap above or snap below
                        if(y<=(dirty[i].bottom - dirty[i].top) / 2 && (dirty[i].top - height) > 0) {
                            y = dirty[i].top - height;
                        } else { // snap down
                            y = dirty[i].bottom;
                        }
                    }
                }
            }

            // Add the draw area to the dirty array
            dirty.push({left:x, right:x+width, top:y-DEFAULT_MARGIN, bottom:y+height+DEFAULT_MARGIN});


            var pointerLength =  (height - 2 * DEFAULT_RADIUS ) / 6;

            context.beginPath();
            context.moveTo(x + DEFAULT_RADIUS, y);
            context.lineTo(x + width - DEFAULT_RADIUS, y);
            context.quadraticCurveTo(x + width, y, x + width, y + DEFAULT_RADIUS);

            if(align=='right') { // point is to the right
                context.lineTo(x + width, y + DEFAULT_RADIUS + pointerLength);
                context.lineTo(x + width + DEFAULT_OFFSET, pointerY);  // point
                context.lineTo(x + width, y + height - DEFAULT_RADIUS - pointerLength);
            }
            context.lineTo(x + width, y + height - DEFAULT_RADIUS);

            context.quadraticCurveTo(x + width, y + height, x + width - DEFAULT_RADIUS, y + height);
            context.lineTo(x + DEFAULT_RADIUS, y + height);
            context.quadraticCurveTo(x, y + height, x, y + height - DEFAULT_RADIUS);

            if(align=='left') { // point is to the left
                context.lineTo(x, y + height - DEFAULT_RADIUS - pointerLength);
                context.lineTo(x - DEFAULT_OFFSET, pointerY); // point
                context.lineTo(x, y + DEFAULT_RADIUS - pointerLength);
            }
            context.lineTo(x, y + DEFAULT_RADIUS);

            context.quadraticCurveTo(x, y, x + DEFAULT_RADIUS, y);
            context.closePath();

            // fill in the balloon background
            context.fill();
            context.stroke();

            // and add the label
            drawAxisLabel(context, axisLabel, x + (width/2), y + (height + fontSize)/2 - 4, 'center', colors.text);

        }

        const BALLOON_COLORS = {
            roll    : {color: 'rgba(255,128,128,0.4)', border: 'rgba(255,128,128,0.6)', text: '#000000'},
            pitch   : {color: 'rgba(128,255,128,0.4)', border: 'rgba(128,255,128,0.6)', text: '#000000'},
            yaw     : {color: 'rgba(128,128,255,0.4)', border: 'rgba(128,128,255,0.6)', text: '#000000'}
        };

        var rcStickElement,
            stickContext;

        if(!rcStickElement) {
            rcStickElement  = $('.rate_curve canvas#rate_curve_layer1').get(0);
            rcStickElement.width = 1000;
            rcStickElement.height = 1000;
        }
        if(!stickContext)   stickContext    = rcStickElement.getContext("2d");

        stickContext.save();

        var
            maxAngularVelRoll   = self.maxAngularVelRollElement.text()  + ' deg/s',
            maxAngularVelPitch  = self.maxAngularVelPitchElement.text() + ' deg/s',
            maxAngularVelYaw    = self.maxAngularVelYawElement.text()   + ' deg/s',
            currentValues       = [],
            balloonsDirty       = [],
            curveHeight         = rcStickElement.height,
            curveWidth          = rcStickElement.width,
            maxAngularVel       = self.rateCurve.maxAngularVel,
            windowScale         = (400 / stickContext.canvas.clientHeight),
            rateScale           = (curveHeight / 2) / maxAngularVel,
            lineScale           = stickContext.canvas.width / stickContext.canvas.clientWidth,
            textScale           = stickContext.canvas.clientHeight / stickContext.canvas.clientWidth;


        stickContext.clearRect(0, 0, curveWidth, curveHeight);

        // calculate the fontSize based upon window scaling
        if(windowScale <= 1) {
            stickContext.font = "24pt Verdana, Arial, sans-serif";
        } else {
            stickContext.font = (24 * windowScale) + "pt Verdana, Arial, sans-serif";
        }

        if(RC.channels[0] && RC.channels[1] && RC.channels[2]) {
            currentValues.push(self.rateCurve.drawStickPosition(RC.channels[0], self.currentRates.roll_rate, self.currentRates.rc_rate, self.currentRates.rc_expo, self.currentRates.superexpo, self.currentRates.deadband, maxAngularVel, stickContext, '#FF8080') + ' deg/s');
            currentValues.push(self.rateCurve.drawStickPosition(RC.channels[1], self.currentRates.roll_rate, self.currentRates.rc_rate, self.currentRates.rc_expo, self.currentRates.superexpo, self.currentRates.deadband, maxAngularVel, stickContext, '#80FF80') + ' deg/s');
            currentValues.push(self.rateCurve.drawStickPosition(RC.channels[2], self.currentRates.yaw_rate, self.currentRates.rc_rate_yaw, self.currentRates.rc_yaw_expo, self.currentRates.superexpo, self.currentRates.yawDeadband, maxAngularVel, stickContext, '#8080FF') + ' deg/s');
        } else {
            currentValues = [];
        }

        stickContext.lineWidth = 1 * lineScale;

        // use a custom scale so that the text does not appear stretched
        stickContext.scale(textScale,1);

        // add the maximum range label
        drawAxisLabel(stickContext, maxAngularVel.toFixed(0) + ' deg/s', ((curveWidth / 2) - 10) / textScale, parseInt(stickContext.font)*1.2, 'right');

        // and then the balloon labels.
        balloonsDirty = []; // reset the dirty balloon draw area (for overlap detection)
        // create an array of balloons to draw
        var balloons = [
            {value: parseInt(maxAngularVelRoll), balloon: function() {drawBalloonLabel(stickContext, maxAngularVelRoll,  curveWidth, rateScale * (maxAngularVel - parseInt(maxAngularVelRoll)),  'right', BALLOON_COLORS.roll, balloonsDirty);}},
            {value: parseInt(maxAngularVelPitch), balloon: function() {drawBalloonLabel(stickContext, maxAngularVelPitch, curveWidth, rateScale * (maxAngularVel - parseInt(maxAngularVelPitch)), 'right', BALLOON_COLORS.pitch, balloonsDirty);}},
            {value: parseInt(maxAngularVelYaw), balloon: function() {drawBalloonLabel(stickContext, maxAngularVelYaw,   curveWidth, rateScale * (maxAngularVel - parseInt(maxAngularVelYaw)),   'right', BALLOON_COLORS.yaw, balloonsDirty);}}
        ];
        // and sort them in descending order so the largest value is at the top always
        balloons.sort(function(a,b) {return (b.value - a.value)});

        // add the current rc values
        if(currentValues[0] && currentValues[1] && currentValues[2]) {
            balloons.push(
                {value: parseInt(currentValues[0]), balloon: function() {drawBalloonLabel(stickContext, currentValues[0], 10, 150, 'none', BALLOON_COLORS.roll, balloonsDirty);}},
                {value: parseInt(currentValues[1]), balloon: function() {drawBalloonLabel(stickContext, currentValues[1], 10, 250, 'none', BALLOON_COLORS.pitch, balloonsDirty);}},
                {value: parseInt(currentValues[2]), balloon: function() {drawBalloonLabel(stickContext, currentValues[2], 10, 350,  'none', BALLOON_COLORS.yaw, balloonsDirty);}}
            );
        }
        // then display them on the chart
        for(var i=0; i<balloons.length; i++) balloons[i].balloon();

        stickContext.restore();

    }
};
