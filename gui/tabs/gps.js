'use strict';

TABS.gps = {};
TABS.gps.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'gps') {
        GUI.active_tab = 'gps';
    }

    function load_html() {
        $('#content').load("./tabs/gps.html", process_html);
    }

    MSP.send_message(MSPCodes.MSP_STATUS, false, false, load_html);
    
    function set_online(){
        $('#connect').hide();
        $('#waiting').show();
        $('#loadmap').hide();
    }
    
    function set_offline(){
        $('#connect').show();
        $('#waiting').hide();
        $('#loadmap').hide();
    }
    
    function process_html() {
        // translate to user-selected languageconsole.log('Online');
        localize();

        function get_raw_gps_data() {
            MSP.send_message(MSPCodes.MSP_RAW_GPS, false, false, get_comp_gps_data);
        }

        function get_comp_gps_data() {
            MSP.send_message(MSPCodes.MSP_COMP_GPS, false, false, get_gpsvinfo_data);
        }

        function get_gpsvinfo_data() {
            MSP.send_message(MSPCodes.MSP_GPS_SV_INFO, false, false, update_ui);
        }

        function update_ui() {
            var lat = GPS_DATA.lat / 10000000;
            var lon = GPS_DATA.lon / 10000000;
            var url = 'https://maps.google.com/?q=' + lat + ',' + lon;

            $('.GPS_info td.fix').html((GPS_DATA.fix) ? chrome.i18n.getMessage('gpsFixTrue') : chrome.i18n.getMessage('gpsFixFalse'));
            $('.GPS_info td.alt').text(GPS_DATA.alt + ' m');
            $('.GPS_info td.lat a').prop('href', url).text(lat.toFixed(4) + ' deg');
            $('.GPS_info td.lon a').prop('href', url).text(lon.toFixed(4) + ' deg');
            $('.GPS_info td.speed').text(GPS_DATA.speed + ' cm/s');
            $('.GPS_info td.sats').text(GPS_DATA.numSat);
            $('.GPS_info td.distToHome').text(GPS_DATA.distanceToHome + ' m');

            // Update GPS Signal Strengths
            var e_ss_table = $('div.GPS_signal_strength table tr:not(.titles)');

            for (var i = 0; i < GPS_DATA.chn.length; i++) {
                var row = e_ss_table.eq(i);

                $('td', row).eq(0).text(GPS_DATA.svid[i]);
                $('td', row).eq(1).text(GPS_DATA.quality[i]);
                $('td', row).eq(2).find('progress').val(GPS_DATA.cno[i]);
            }
            

            var message = {
                action: 'center',
                lat: lat,
                lon: lon,
            };

            var frame = document.getElementById('map');
            if (navigator.onLine) {
                $('#connect').hide();

                //if(lat != 0 && lon != 0){
                if(GPS_DATA.fix){
                   frame.contentWindow.postMessage(message, '*');
                   $('#loadmap').show();
                   $('#waiting').hide();
                }else{
                   $('#loadmap').hide();
                   $('#waiting').show();
                }
            }else{
                $('#connect').show();
                $('#waiting').hide(); 
                $('#loadmap').hide();
            }
        }

        // enable data pulling
        GUI.interval_add('gps_pull', function gps_update() {
            // avoid usage of the GPS commands until a GPS sensor is detected for targets that are compiled without GPS support.
            if (!have_sensor(CONFIG.activeSensors, 'gps')) {
                //return;
            }
            
            get_raw_gps_data();
        }, 75, true);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSPCodes.MSP_STATUS);
        }, 250, true);


        //check for internet connection on load
        if (navigator.onLine) {
            console.log('Online');
            set_online();
        } else {
            console.log('Offline');
            set_offline();
        }

        $("#check").on('click',function(){
            if (navigator.onLine) {
                console.log('Online');
                set_online();
            } else {
                console.log('Offline');
                set_offline();
            }
        });

        var frame = document.getElementById('map');

        $('#zoom_in').click(function() {
            console.log('zoom in');
            var message = {
                action: 'zoom_in',
            };
            frame.contentWindow.postMessage(message, '*');
        });
        
        $('#zoom_out').click(function() {
            console.log('zoom out');
            var message = {
                action: 'zoom_out'
            };
            frame.contentWindow.postMessage(message, '*');
        });
 
        GUI.content_ready(callback);
    }

};


 
TABS.gps.cleanup = function (callback) {
    if (callback) callback();
};
