'use strict';

tabs.pid_tuning = {};
tabs.pid_tuning.initialize = function(callback) {
    GUI.active_tab_ref = this;
    GUI.active_tab = 'pid_tuning';
    googleAnalytics.sendAppView('PID Tuning');

    // requesting MSP_STATUS manually because it contains CONFIG.profile
    MSP.send_message(MSP_codes.MSP_STATUS, false, false, get_pid_names);

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

        // Fill in the data from PIDs array
        var i = 0;
        $('.pid_tuning .ROLL input').each(function() {
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
        $('.pid_tuning .PITCH input').each(function() {
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
        $('.pid_tuning .YAW input').each(function() {
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
        $('.pid_tuning .ALT input').each(function() {
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
        $('.pid_tuning .Pos input').each(function() {
            $(this).val(PIDs[4][i++].toFixed(2));
        });

        i = 0;
        $('.pid_tuning .PosR input').each(function() {
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
        $('.pid_tuning .NavR input').each(function() {
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
        $('.pid_tuning .LEVEL input').each(function() {
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
        $('.pid_tuning .MAG input').each(function() {
            $(this).val(PIDs[8][i++].toFixed(1));
        });

        i = 0;
        $('.pid_tuning .Vario input').each(function() {
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
        $('.rate-tpa input[name="yaw"]').val(RC_tuning.yaw_rate.toFixed(2));
        $('.rate-tpa input[name="tpa"]').val(RC_tuning.dynamic_THR_PID.toFixed(2));

        // Fill in currently selected profile
        $('input[name="profile"]').val(CONFIG.profile + 1); // +1 because the range is 0-2

        // UI Hooks
        $('input[name="profile"]').change(function() {
            var profile = parseInt($(this).val());
            MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [profile - 1], false, function() {
                GUI.log(chrome.i18n.getMessage('pidTuningLoadedProfile', [profile]));

                GUI.tab_switch_cleanup(function() {
                    tabs.pid_tuning.initialize();
                });
            });
        });

        $('a.refresh').click(function() {
            GUI.tab_switch_cleanup(function() {
                GUI.log(chrome.i18n.getMessage('pidTuningDataRefreshed'));

                tabs.pid_tuning.initialize();
            });
        });

        $('a.update').click(function() {
            // Catch all the changes and stuff the inside PIDs array
            var i = 0;
            $('table.pid_tuning tr.ROLL input').each(function() {
                PIDs[0][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.PITCH input').each(function() {
                PIDs[1][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.YAW input').each(function() {
                PIDs[2][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.ALT input').each(function() {
                PIDs[3][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.Vario input').each(function() {
                PIDs[9][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.Pos input').each(function() {
                PIDs[4][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.PosR input').each(function() {
                PIDs[5][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.NavR input').each(function() {
                PIDs[6][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.LEVEL input').each(function() {
                PIDs[7][i++] = parseFloat($(this).val());
            });

            i = 0;
            $('table.pid_tuning tr.MAG input').each(function() {
                PIDs[8][i++] = parseFloat($(this).val());
            });

            var PID_buffer_out = new Array();
            for (var i = 0, needle = 0; i < PIDs.length; i++, needle += 3) {
                switch (i) {
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                    case 7:
                    case 8:
                    case 9:
                        PID_buffer_out[needle]     = parseInt(PIDs[i][0] * 10);
                        PID_buffer_out[needle + 1] = parseInt(PIDs[i][1] * 1000);
                        PID_buffer_out[needle + 2] = parseInt(PIDs[i][2]);
                        break;
                    case 4:
                        PID_buffer_out[needle]     = parseInt(PIDs[i][0] * 100);
                        PID_buffer_out[needle + 1] = parseInt(PIDs[i][1] * 100);
                        PID_buffer_out[needle + 2] = parseInt(PIDs[i][2]);
                        break;
                    case 5:
                    case 6:
                        PID_buffer_out[needle]     = parseInt(PIDs[i][0] * 10);
                        PID_buffer_out[needle + 1] = parseInt(PIDs[i][1] * 100);
                        PID_buffer_out[needle + 2] = parseInt(PIDs[i][2] * 1000);
                        break;
                }
            }

            // Send over the PID changes
            MSP.send_message(MSP_codes.MSP_SET_PID, PID_buffer_out, false, send_rc_tuning_changes);

            function send_rc_tuning_changes() {
                // catch RC_tuning changes
                RC_tuning.roll_pitch_rate = parseFloat($('.rate-tpa input[name="roll-pitch"]').val());
                RC_tuning.yaw_rate = parseFloat($('.rate-tpa input[name="yaw"]').val());
                RC_tuning.dynamic_THR_PID = parseFloat($('.rate-tpa input[name="tpa"]').val());

                var RC_tuning_buffer_out = new Array();
                RC_tuning_buffer_out[0] = parseInt(RC_tuning.RC_RATE * 100);
                RC_tuning_buffer_out[1] = parseInt(RC_tuning.RC_EXPO * 100);
                RC_tuning_buffer_out[2] = parseInt(RC_tuning.roll_pitch_rate * 100);
                RC_tuning_buffer_out[3] = parseInt(RC_tuning.yaw_rate * 100);
                RC_tuning_buffer_out[4] = parseInt(RC_tuning.dynamic_THR_PID * 100);
                RC_tuning_buffer_out[5] = parseInt(RC_tuning.throttle_MID * 100);
                RC_tuning_buffer_out[6] = parseInt(RC_tuning.throttle_EXPO * 100);

                // Send over the RC_tuning changes
                MSP.send_message(MSP_codes.MSP_SET_RC_TUNING, RC_tuning_buffer_out, false, save_to_eeprom);
            }

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function() {
                    GUI.log(chrome.i18n.getMessage('pidTuningEepromSaved'));
                });
            }
        });

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

tabs.pid_tuning.cleanup = function(callback) {
    if (callback) callback();
}