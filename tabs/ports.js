'use strict';

TABS.ports = {};

TABS.ports.initialize = function (callback, scrollPosition) {
    var self = this;

    var board_definition = {};

    var functionRules = [
         {name: 'MSP',          groups: ['data', 'msp'], maxPorts: 2},
         {name: 'GPS',          groups: ['gps'], maxPorts: 1},
         {name: 'FrSky',        groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1},
         {name: 'HoTT',         groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1},
         {name: 'MSP',          groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1},
         {name: 'SmartPort',    groups: ['telemetry'], maxPorts: 1},
         {name: 'Serial RX',    groups: ['rx'], maxPorts: 1},
         {name: 'Blackbox',     groups: ['logging', 'blackbox'], sharableWith: ['msp'], notSharableWith: ['telemetry'], maxPorts: 1},
    ];

    var mspBaudRates = [
        '9600',
        '19200',
        '38400',
        '57600',
        '115200'
    ];

    var gpsBaudRates = [
        '9600',
        '19200',
        '38400',
        '57600',
        '115200'
    ];

    var telemetryBaudRates = [
        'AUTO',
        '9600',
        '19200',
        '38400',
        '57600',
        '115200'
    ];

    var blackboxBaudRates = [
       '115200'
    ];

    var columns = ['data', 'logging', 'gps', 'telemetry', 'rx'];

    if (GUI.active_tab != 'ports') {
        GUI.active_tab = 'ports';
        googleAnalytics.sendAppView('Ports');
    }

    load_configuration_from_fc();
    
    function load_configuration_from_fc() {
        MSP.send_message(MSP_codes.MSP_CF_SERIAL_CONFIG, false, false, on_configuration_loaded_handler);
        
        function on_configuration_loaded_handler() {
            $('#content').load("./tabs/ports.html", on_tab_loaded_handler);
            
            board_definition = BOARD.find_board_definition(CONFIG.boardIdentifier);
            console.log('Using board definition', board_definition);
        }
    }

    function update_ui() {
        
        
        var portIdentifierToNameMapping = {
           0: 'UART1',
           1: 'UART2',
           2: 'UART3',
           3: 'UART4',
           20: 'USB VCP',
           30: 'SOFTSERIAL1',
           31: 'SOFTSERIAL2'
        };

        var gps_baudrate_e = $('select.gps_baudrate');
        for (var i = 0; i < gpsBaudRates.length; i++) {
            gps_baudrate_e.append('<option value="' + gpsBaudRates[i] + '">' + gpsBaudRates[i] + '</option>');
        }

        var msp_baudrate_e = $('select.msp_baudrate');
        for (var i = 0; i < mspBaudRates.length; i++) {
            msp_baudrate_e.append('<option value="' + mspBaudRates[i] + '">' + mspBaudRates[i] + '</option>');
        }

        var telemetry_baudrate_e = $('select.telemetry_baudrate');
        for (var i = 0; i < telemetryBaudRates.length; i++) {
            telemetry_baudrate_e.append('<option value="' + telemetryBaudRates[i] + '">' + telemetryBaudRates[i] + '</option>');
        }

        var blackbox_baudrate_e = $('select.blackbox_baudrate');
        for (var i = 0; i < blackboxBaudRates.length; i++) {
            blackbox_baudrate_e.append('<option value="' + blackboxBaudRates[i] + '">' + blackboxBaudRates[i] + '</option>');
        }

        for (var columnIndex = 0; columnIndex < columns.length; columnIndex++) {
            var column = columns[columnIndex];
            
            var functions_e = $('#tab-ports-templates .functionsCell-' + column);
            
            for (var i = 0; i < functionRules.length; i++) {
                if (functionRules[i].groups.indexOf(column) >= 0) {
                    functions_e.prepend('<span class="function"><input type="checkbox" id="checkbox-' + columnIndex + '-' + i + '" value="' + i + '" /><label for="checkbox-' + columnIndex + '-' + i + '"> ' + functionRules[i].name + '</label></span>');
                }
                
            }
        }

        var ports_e = $('.tab-ports .ports');
        var port_configuration_template_e = $('#tab-ports-templates .portConfiguration');
        
        for (var portIndex = 0; portIndex < SERIAL_CONFIG.ports.length; portIndex++) {
            var port_configuration_e = port_configuration_template_e.clone();
            
            var serialPort = SERIAL_CONFIG.ports[portIndex];

            // TODO check functions
            // TODO set baudrate
            port_configuration_e.find('.identifier').text(portIdentifierToNameMapping[serialPort.identifier])
            
            port_configuration_e.data('index', portIndex);
            port_configuration_e.data('port', serialPort);
            
            ports_e.find('tbody').append(port_configuration_e);
        }
    }
    
    function on_tab_loaded_handler() {

        localize();
        
        update_ui();

        $('a.save').click(on_save_handler);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function status_pull() {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }

    function on_save_handler() {
        
        // update configuration based on current ui state
        var ports_e = $('.tab-ports .portConfiguration').each(function () {
            var portIndex = $(this).data('index');
            
            SERIAL_CONFIG.ports[portIndex].scenario = parseInt($(this).find('select').val());
        });
        

        MSP.send_message(MSP_codes.MSP_SET_CF_SERIAL_CONFIG, MSP.crunch(MSP_codes.MSP_SET_CF_SERIAL_CONFIG), false, save_to_eeprom);

        function save_to_eeprom() {
            MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, on_saved_handler);
        }

        function on_saved_handler() {
            GUI.log(chrome.i18n.getMessage('configurationEepromSaved'));

            GUI.tab_switch_cleanup(function() {
                MSP.send_message(MSP_codes.MSP_SET_REBOOT, false, false, on_reboot_success_handler);
            });
        }

        function on_reboot_success_handler() {
            GUI.log(chrome.i18n.getMessage('deviceRebooting'));

            var rebootTimeoutDelay = 1500;  // seems to be just the right amount of delay to prevent data request timeouts
            
            GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('deviceReady'));
                    TABS.ports.initialize(false, $('#content').scrollTop());
                });
            }, rebootTimeoutDelay); 
        }
    }
};

TABS.ports.cleanup = function (callback) {
    if (callback) callback();
};