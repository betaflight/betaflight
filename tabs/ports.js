'use strict';

TABS.ports = {};

TABS.ports.initialize = function (callback, scrollPosition) {
    var self = this;

    var board_definition = {};
    
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

    function addSerialPortScenarios() {
        
        var scenarioNames = [
            'Unused',
            'MSP, CLI, Telemetry (when armed), GPS Passthrough',
            'GPS',
            'Serial RX',
            'Telemetry',
            'MSP, CLI, GPS Passthrough',
            'CLI',
            'GPS Passthrough',
            'MSP',
            'SmartPort Telemetry',
            'Blackbox',
            'MSP, CLI, Blackbox (when armed), GPS Passthrough'
        ];
        
        var portIdentifierToNameMapping = {
           0: 'UART1',
           1: 'UART2',
           2: 'UART3',
           3: 'UART4',
           20: 'USB VCP',
           30: 'SOFTSERIAL1',
           31: 'SOFTSERIAL2'
        };
        
        var scenario_e = $('#tab-ports-templates select.scenario');
        
        for (var i = 0; i < scenarioNames.length; i++) {
            scenario_e.append('<option value="' + i + '">' + scenarioNames[i] + '</option>');
        }
        
        var ports_e = $('.tab-ports .ports');
        var port_configuration_template_e = $('#tab-ports-templates .portConfiguration');
        
        for (var portIndex = 0; portIndex < SERIAL_CONFIG.ports.length; portIndex++) {
            var port_configuration_e = port_configuration_template_e.clone();
            
            var serialPort = SERIAL_CONFIG.ports[portIndex];
            
            port_configuration_e.find('select').val(serialPort.scenario);
            port_configuration_e.find('.identifier').text(portIdentifierToNameMapping[serialPort.identifier])
            
            port_configuration_e.data('index', portIndex);
            port_configuration_e.data('port', serialPort);
            
            ports_e.find('tbody').append(port_configuration_e);
        }
    }
    
    function on_tab_loaded_handler() {

        localize();
        
        addSerialPortScenarios();

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