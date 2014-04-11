function tab_initialize_gps () {
    ga_tracker.sendAppView('GPS Page');
    GUI.active_tab = 'gps';

    send_message(MSP_codes.MSP_RAW_GPS, false, false, load_html);

    function load_html() {
        $('#content').load("./tabs/gps.html", process_html);
    }

    function process_html() {
        function get_raw_gps_data() {
            send_message(MSP_codes.MSP_RAW_GPS, false, false, get_gpsvinfo_data);
        }

        function get_gpsvinfo_data() {
            send_message(MSP_codes.MSP_GPSSVINFO, false, false, update_ui);
        }

        function update_ui() {
            $('.GPS_info td.alt').html(GPS_DATA.alt + ' m');
            $('.GPS_info td.lat').html((GPS_DATA.lat / 10000000).toFixed(4) + ' deg');
            $('.GPS_info td.lon').html((GPS_DATA.lon / 10000000).toFixed(4) + ' deg');
            $('.GPS_info td.speed').html(GPS_DATA.speed + ' cm/s');
            $('.GPS_info td.sats').html(GPS_DATA.numSat);
            $('.GPS_info td.distToHome').html(GPS_DATA.distanceToHome + ' m');

            // Update GPS Signal Strengths
            var e_ss_table = $('div.GPS_signal_strength table tr:not(.titles)');

            for (var i = 0; i < GPS_DATA.chn.length; i++) {
                var row = e_ss_table.eq(i);

                $('td', row).eq(0).html(GPS_DATA.svid[i]);
                $('td', row).eq(1).html(GPS_DATA.quality[i]);
                $('td', row).eq(2).find('progress').val(GPS_DATA.cno[i]);
            }
        }

        // enable data pulling
        GUI.interval_add('gps_pull', get_raw_gps_data, 75, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function() {
            send_message(MSP_codes.MSP_STATUS);
        }, 250, true);
    }
}