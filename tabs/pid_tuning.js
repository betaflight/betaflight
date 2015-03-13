'use strict';

TABS.pid_tuning = {
    controllerChanged: true
};

TABS.pid_tuning.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'pid_tuning') {
        GUI.active_tab = 'pid_tuning';
        googleAnalytics.sendAppView('PID Tuning');
    }

    function get_pid_controller() {
        if (GUI.canChangePidController) {
            MSP.send_message(MSP_codes.MSP_PID_CONTROLLER, false, false, get_pid_names);
        } else {
            get_pid_names();
        }
    }

    function get_pid_names() {
        MSP.send_message(MSP_codes.MSP_PIDNAMES, false, false, get_pid_data);
    }

    function get_pid_data() {
        MSP.send_message(MSP_codes.MSP_PID, false, false, get_rc_tuning_data);
    }

    function get_rc_tuning_data() {
        MSP.send_message(MSP_codes.MSP_RC_TUNING, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/pid_tuning.html", process_html);
    }

    // requesting MSP_STATUS manually because it contains CONFIG.profile
    MSP.send_message(MSP_codes.MSP_STATUS, false, false, get_pid_controller);

    function pid_and_rc_to_form() {
        // Fill in the data from PIDs array
        var i = 0;
        $('.pid_tuning .ROLL input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[0][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[0][i++].toFixed(3));
                    break;
                case 2:
                    $(this).val(PIDs[0][i++].toFixed(0));
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .PITCH input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[1][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[1][i++].toFixed(3));
                    break;
                case 2:
                    $(this).val(PIDs[1][i++].toFixed(0));
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .YAW input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[2][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[2][i++].toFixed(3));
                    break;
                case 2:
                    $(this).val(PIDs[2][i++].toFixed(0));
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .ALT input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[3][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[3][i++].toFixed(3));
                    break;
                case 2:
                    $(this).val(PIDs[3][i++].toFixed(0));
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .Pos input').each(function () {
            $(this).val(PIDs[4][i++].toFixed(2));
        });

        i = 0;
        $('.pid_tuning .PosR input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[5][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[5][i++].toFixed(2));
                    break;
                case 2:
                    $(this).val(PIDs[5][i++].toFixed(3));
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .NavR input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[6][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[6][i++].toFixed(2));
                    break;
                case 2:
                    $(this).val(PIDs[6][i++].toFixed(3));
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .LEVEL input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[7][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[7][i++].toFixed(3));
                    break;
                case 2:
                    $(this).val(PIDs[7][i++].toFixed(0));
                    break;
            }
        });

        i = 0;
        $('.pid_tuning .MAG input').each(function () {
            $(this).val(PIDs[8][i++].toFixed(1));
        });

        i = 0;
        $('.pid_tuning .Vario input').each(function () {
            switch (i) {
                case 0:
                    $(this).val(PIDs[9][i++].toFixed(1));
                    break;
                case 1:
                    $(this).val(PIDs[9][i++].toFixed(3));
                    break;
                case 2:
                    $(this).val(PIDs[9][i++].toFixed(0));
                    break;
            }
        });

        // Fill in data from RC_tuning object
        $('.rate-tpa input[name="roll-pitch"]').val(RC_tuning.roll_pitch_rate.toFixed(2));
        $('.rate-tpa input[name="roll"]').val(RC_tuning.roll_rate.toFixed(2));
        $('.rate-tpa input[name="pitch"]').val(RC_tuning.pitch_rate.toFixed(2));
        $('.rate-tpa input[name="yaw"]').val(RC_tuning.yaw_rate.toFixed(2));
        $('.rate-tpa input[name="tpa"]').val(RC_tuning.dynamic_THR_PID.toFixed(2));
        $('.rate-tpa input[name="tpa-breakpoint"]').val(RC_tuning.dynamic_THR_breakpoint);
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
        $('table.pid_tuning tr.LEVEL input').each(function () {
            PIDs[7][i++] = parseFloat($(this).val());
        });

        i = 0;
        $('table.pid_tuning tr.MAG input').each(function () {
            PIDs[8][i++] = parseFloat($(this).val());
        });

        // catch RC_tuning changes
        RC_tuning.roll_pitch_rate = parseFloat($('.rate-tpa input[name="roll-pitch"]').val());
        RC_tuning.roll_rate = parseFloat($('.rate-tpa input[name="roll"]').val());
        RC_tuning.pitch_rate = parseFloat($('.rate-tpa input[name="pitch"]').val());
        RC_tuning.yaw_rate = parseFloat($('.rate-tpa input[name="yaw"]').val());
        RC_tuning.dynamic_THR_PID = parseFloat($('.rate-tpa input[name="tpa"]').val());
        RC_tuning.dynamic_THR_breakpoint = parseInt($('.rate-tpa input[name="tpa-breakpoint"]').val());
    }

    function process_html() {
        // translate to user-selected language
        localize();

        // Fill in the names from PID_names array
        // this needs to be reworked, but will do for now
        $('.pid_tuning tr:eq(1) td:first').text(PID_names[0]);
        $('.pid_tuning tr:eq(2) td:first').text(PID_names[1]);
        $('.pid_tuning tr:eq(3) td:first').text(PID_names[2]);
        $('.pid_tuning tr:eq(4) td:first').text(PID_names[3]);
        $('.pid_tuning tr:eq(5) td:first').text(PID_names[9]);
        $('.pid_tuning tr:eq(6) td:first').text(PID_names[4]);
        $('.pid_tuning tr:eq(7) td:first').text(PID_names[5]);
        $('.pid_tuning tr:eq(8) td:first').text(PID_names[6]);
        $('.pid_tuning tr:eq(9) td:first').text(PID_names[7]);
        $('.pid_tuning tr:eq(10) td:first').text(PID_names[8]);

        pid_and_rc_to_form();

        var pidController_e = $('select[name="controller"]');
        var profile_e = $('select[name="profile"]');
        var form_e = $('#pid-tuning');

        if (GUI.canChangePidController) {
            pidController_e.val(PID.controller);
        } else {
            GUI.log(chrome.i18n.getMessage('pidTuningUpgradeFirmwareToChangePidController', [CONFIG.apiVersion, CONFIGURATOR.pidControllerChangeMinApiVersion]));

            pidController_e.empty();
            pidController_e.append('<option value="">Unknown</option>');

            pidController_e.prop('disabled', true);
        }
        
        if (CONFIG.apiVersion < 1.7) {
            $('.rate-tpa .tpa-breakpoint').hide();
            $('.rate-tpa .roll').hide();
            $('.rate-tpa .pitch').hide();
        } else {
            $('.rate-tpa .roll-pitch').hide();
        }

        // Fill in currently selected profile

        profile_e.val(CONFIG.profile);

        // UI Hooks
        profile_e.change(function () {
            var profile = parseInt($(this).val());
            MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [profile], false, function () {
                GUI.log(chrome.i18n.getMessage('pidTuningLoadedProfile', [profile + 1]));

                GUI.tab_switch_cleanup(function () {
                    TABS.pid_tuning.initialize();
                });
            });
        });

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


        // update == save.
        $('a.update').click(function () {
            form_to_pid_and_rc();

            function send_pids() {
                if (!TABS.pid_tuning.controllerChanged) {
                    MSP.send_message(MSP_codes.MSP_SET_PID, MSP.crunch(MSP_codes.MSP_SET_PID), false, send_rc_tuning_changes);
                }
            }

            function send_rc_tuning_changes() {
                MSP.send_message(MSP_codes.MSP_SET_RC_TUNING, MSP.crunch(MSP_codes.MSP_SET_RC_TUNING), false, save_to_eeprom);
            }

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('pidTuningEepromSaved'));
                });
            }

            if (GUI.canChangePidController && TABS.pid_tuning.controllerChanged) {
                PID.controller = pidController_e.val();
                MSP.send_message(MSP_codes.MSP_SET_PID_CONTROLLER, MSP.crunch(MSP_codes.MSP_SET_PID_CONTROLLER), false, function () {
                    MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('pidTuningEepromSaved'));
                    });
                    TABS.pid_tuning.initialize();
                });
            } else {
                send_pids();
            }
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) {
            callback();
        }
    }
};

TABS.pid_tuning.cleanup = function (callback) {
    if (callback) {
        callback();
    }
};
