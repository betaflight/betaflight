'use strict';

TABS.ports = {};

TABS.ports.initialize = function (callback, scrollPosition) {
    var self = this;

    var board_definition = {};

    var functionRules = [
         {name: 'MSP',                  groups: ['data', 'msp'], maxPorts: 2},
         {name: 'GPS',                  groups: ['gps'], maxPorts: 1},
         {name: 'TELEMETRY_FRSKY',      groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1},
         {name: 'TELEMETRY_HOTT',       groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1},
         {name: 'TELEMETRY_SMARTPORT',  groups: ['telemetry'], maxPorts: 1},
         {name: 'RX_SERIAL',            groups: ['rx'], maxPorts: 1},
         {name: 'BLACKBOX',             groups: ['logging', 'blackbox'], sharableWith: ['msp'], notSharableWith: ['telemetry'], maxPorts: 1},
    ];

    if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
        var ltmFunctionRule = {name: 'TELEMETRY_LTM',        groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1};
        functionRules.push(ltmFunctionRule);
    } else {
        var mspFunctionRule = {name: 'TELEMETRY_MSP',        groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1};
        functionRules.push(mspFunctionRule);
    }

    if (semver.gte(CONFIG.apiVersion, "1.18.0")) {
        var mavlinkFunctionRule = {name: 'TELEMETRY_MAVLINK',    groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['blackbox'], maxPorts: 1};
        functionRules.push(mavlinkFunctionRule);
    }

    for (var i = 0; i < functionRules.length; i++) {
        functionRules[i].displayName = chrome.i18n.getMessage('portsFunction_' + functionRules[i].name);
    }

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
        '19200',
        '38400',
        '57600',
        '115200',
        '230400',
        '250000',
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
        
        if (semver.lt(CONFIG.apiVersion, "1.6.0")) {
            
            $(".tab-ports").removeClass("supported");
            return;
        }
        
        $(".tab-ports").addClass("supported");
        
        var portIdentifierToNameMapping = {
           0: 'UART1',
           1: 'UART2',
           2: 'UART3',
           3: 'UART4',
           4: 'UART5',
           5: 'UART6',
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

        var ports_e = $('.tab-ports .ports');
        var port_configuration_template_e = $('#tab-ports-templates .portConfiguration');
        
        for (var portIndex = 0; portIndex < SERIAL_CONFIG.ports.length; portIndex++) {
            var port_configuration_e = port_configuration_template_e.clone();
            var serialPort = SERIAL_CONFIG.ports[portIndex];
            
            port_configuration_e.data('serialPort', serialPort);
            
            var msp_baudrate_e = port_configuration_e.find('select.msp_baudrate');
            msp_baudrate_e.val(serialPort.msp_baudrate);

            var telemetry_baudrate_e = port_configuration_e.find('select.telemetry_baudrate');
            telemetry_baudrate_e.val(serialPort.telemetry_baudrate);

            var gps_baudrate_e = port_configuration_e.find('select.gps_baudrate');
            gps_baudrate_e.val(serialPort.gps_baudrate);

            var blackbox_baudrate_e = port_configuration_e.find('select.blackbox_baudrate');
            blackbox_baudrate_e.val(serialPort.blackbox_baudrate);

            port_configuration_e.find('.identifier').text(portIdentifierToNameMapping[serialPort.identifier])
            
            port_configuration_e.data('index', portIndex);
            port_configuration_e.data('port', serialPort);


            for (var columnIndex = 0; columnIndex < columns.length; columnIndex++) {
                var column = columns[columnIndex];
                
                var functions_e = $(port_configuration_e).find('.functionsCell-' + column);
                
                for (var i = 0; i < functionRules.length; i++) {
                    var functionRule = functionRules[i];
                    var functionName = functionRule.name;
                    
                    if (functionRule.groups.indexOf(column) == -1) {
                        continue;
                    }
                    
                    var select_e;
                    if (column != 'telemetry') {
                        var checkboxId = 'functionCheckbox-' + portIndex + '-' + columnIndex + '-' + i;
                        functions_e.prepend('<span class="function"><input type="checkbox" class="togglemedium" id="' + checkboxId + '" value="' + functionName + '" /><label for="' + checkboxId + '"> ' + functionRule.displayName + '</label></span>');

                        if (serialPort.functions.indexOf(functionName) >= 0) {
                            var checkbox_e = functions_e.find('#' + checkboxId);
                            checkbox_e.prop("checked", true);
                        }
                        
                    } else {
                        
                        var selectElementName = 'function-' + column;
                        var selectElementSelector = 'select[name=' + selectElementName + ']';
                        select_e = functions_e.find(selectElementSelector);
                        
                        if (select_e.size() == 0) {
                            functions_e.prepend('<span class="function"><select name="' + selectElementName + '" /></span>');
                            select_e = functions_e.find(selectElementSelector);
                            var disabledText = chrome.i18n.getMessage('portsTelemetryDisabled');
                            select_e.append('<option value="">' + disabledText + '</option>');
                        }
                        select_e.append('<option value="' + functionName + '">' + functionRule.displayName + '</option>');

                        if (serialPort.functions.indexOf(functionName) >= 0) {
                            select_e.val(functionName);
                        }
                    }
                }
            }

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

        GUI.content_ready(callback);
    }

   function on_save_handler() {
        
        // update configuration based on current ui state
        SERIAL_CONFIG.ports = [];

        var ports_e = $('.tab-ports .portConfiguration').each(function (portConfiguration_e) {
            
            var portConfiguration_e = this;
            
            var oldSerialPort = $(this).data('serialPort');
            
            var functions = $(portConfiguration_e).find('input:checkbox:checked').map(function() {
                return this.value;
            }).get();
            
            var telemetryFunction = $(portConfiguration_e).find('select[name=function-telemetry]').val();
            if (telemetryFunction) {
                functions.push(telemetryFunction);
            }
            
            var serialPort = {
                functions: functions,
                msp_baudrate: $(portConfiguration_e).find('.msp_baudrate').val(),
                telemetry_baudrate: $(portConfiguration_e).find('.telemetry_baudrate').val(),
                gps_baudrate: $(portConfiguration_e).find('.gps_baudrate').val(),
                blackbox_baudrate: $(portConfiguration_e).find('.blackbox_baudrate').val(),
                identifier: oldSerialPort.identifier
            };
            SERIAL_CONFIG.ports.push(serialPort);
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

            if (BOARD.find_board_definition(CONFIG.boardIdentifier).vcp) { // VCP-based flight controls may crash old drivers, we catch and reconnect
                $('a.connect').click();
                GUI.timeout_add('start_connection',function start_connection() {
                    $('a.connect').click();
                },2500);
            } else {
                GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                    MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('deviceReady'));
                        TABS.ports.initialize(false, $('#content').scrollTop());
                    });
               },  1500); // seems to be just the right amount of delay to prevent data request timeouts
            }
        }
    }
};

TABS.ports.cleanup = function (callback) {
    if (callback) callback();
};
