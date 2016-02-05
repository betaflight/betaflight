'use strict';

TABS.transponder = {
    available: false
};

TABS.transponder.initialize = function (callback, scrollPosition) {
    var self = this;

    if (GUI.active_tab != 'transponder') {
        GUI.active_tab = 'transponder';
        googleAnalytics.sendAppView('Transponder');
    }

    // transponder supported added in MSP API Version 1.16.0
    TABS.transponder.available = semver.gte(CONFIG.apiVersion, "1.16.0");
    
    if (!TABS.transponder.available) {
        load_html();
        return;
    }

    function load_html() {
        $('#content').load("./tabs/transponder.html", process_html);
    }

    // get the transponder data and a flag to see if transponder support is enabled on the FC
    MSP.send_message(MSP_codes.MSP_TRANSPONDER_CONFIG, false, false, load_html);

    // Convert a hex string to a byte array
    function hexToBytes(hex) {
        for (var bytes = [], c = 0; c < hex.length; c += 2)
        bytes.push(~parseInt(hex.substr(c, 2), 16));
        return bytes;
    }

    function pad(n, width) {
        n = n + '';
        return n.length >= width ? n : new Array(width - n.length + 1).join('0') + n;
    }
    
    // Convert a byte array to a hex string
    function bytesToHex(bytes) {
        for (var hex = [], i = 0; i < bytes.length; i++) {
            hex.push(pad(((~bytes[i]) & 0xFF).toString(16),2));
        }
        return hex.join("").toUpperCase();
    }
    function process_html() {
        // translate to user-selected language
        localize();
        
        $(".tab-transponder")
        .toggleClass("transponder-supported", TABS.transponder.available && TRANSPONDER.supported);
        
        if (TABS.transponder.available) {
        
            var data = bytesToHex(TRANSPONDER.data); 
                
            $('input[name="data"]').val(data);
            $('input[name="data"]').prop('maxLength', data.length);
    
            $('a.save').click(function () {
                
                
                // gather data that doesn't have automatic change event bound
                
                var dataString = $('input[name="data"]').val();
                var expectedLength = TRANSPONDER.data.length;
                var hexRegExp = new RegExp('[0-9a-fA-F]{' + (expectedLength * 2) + '}', 'gi');
                if (!dataString.match(hexRegExp)) {
                    GUI.log(chrome.i18n.getMessage('transponderDataInvalid')); 
                    return;
                }
                
                TRANSPONDER.data = hexToBytes(dataString);
                
    
                //
                // send data to FC
                //
                function save_transponder_config() {
                    MSP.send_message(MSP_codes.MSP_SET_TRANSPONDER_CONFIG, MSP.crunch(MSP_codes.MSP_SET_TRANSPONDER_CONFIG), false, save_to_eeprom);
                }
                function save_to_eeprom() {
                    MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('transponderEepromSaved')); 
                    });
                }
                
                save_transponder_config();
            });
        }
        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        GUI.content_ready(callback);
    }
};

TABS.transponder.cleanup = function (callback) {
    if (callback) callback();
};
