'use strict';

// code below is highly experimental, although it runs fine on latest firmware
// the data inside nested objects needs to be verified if deep copy works properly
function configuration_backup(callback) {
    var activeProfile = null;

    var configuration = {
        'generatedBy': chrome.runtime.getManifest().version,
        'apiVersion': CONFIG.apiVersion,
        'profiles': [],
    };
    
    var profileSpecificData = [
        MSPCodes.MSP_PID_CONTROLLER,
        MSPCodes.MSP_PID,
        MSPCodes.MSP_RC_TUNING,
        MSPCodes.MSP_ACC_TRIM,
        MSPCodes.MSP_SERVO_CONFIGURATIONS,
        MSPCodes.MSP_MODE_RANGES,
        MSPCodes.MSP_ADJUSTMENT_RANGES
    ];

    function update_profile_specific_data_list() {
        if (semver.lt(CONFIG.apiVersion, "1.12.0")) {
            profileSpecificData.push(MSPCodes.MSP_CHANNEL_FORWARDING);
         } else {            
            profileSpecificData.push(MSPCodes.MSP_SERVO_MIX_RULES);
        }
        if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
            profileSpecificData.push(MSPCodes.MSP_RC_DEADBAND);
        }
    }
    
    update_profile_specific_data_list();

    MSP.send_message(MSPCodes.MSP_STATUS, false, false, function () {
        activeProfile = CONFIG.profile;
        select_profile();
    });

    function select_profile() {
        if (activeProfile > 0) {
            MSP.send_message(MSPCodes.MSP_SELECT_SETTING, [0], false, fetch_specific_data);
        } else {
            fetch_specific_data();
        }
    }

    function fetch_specific_data() {
        var fetchingProfile = 0,
            codeKey = 0;

        function fetch_specific_data_item() {
            if (fetchingProfile < CONFIG.numProfiles) {
                MSP.send_message(profileSpecificData[codeKey], false, false, function () {
                    codeKey++;

                    if (codeKey < profileSpecificData.length) {
                        fetch_specific_data_item();
                    } else {
                        configuration.profiles.push({
                            'PID': jQuery.extend(true, {}, PID),
                            'PIDs': jQuery.extend(true, [], PIDs),
                            'RC': jQuery.extend(true, {}, RC_tuning),
                            'AccTrim': jQuery.extend(true, [], CONFIG.accelerometerTrims),
                            'ServoConfig': jQuery.extend(true, [], SERVO_CONFIG),
                            'ServoRules': jQuery.extend(true, [], SERVO_RULES),
                            'ModeRanges': jQuery.extend(true, [], MODE_RANGES),
                            'AdjustmentRanges': jQuery.extend(true, [], ADJUSTMENT_RANGES)
                        });

                        if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                            configuration.profiles[fetchingProfile].RCdeadband = jQuery.extend(true, {}, RC_deadband);
                        }
                        codeKey = 0;
                        fetchingProfile++;

                        MSP.send_message(MSPCodes.MSP_SELECT_SETTING, [fetchingProfile], false, fetch_specific_data_item);
                    }
                });
            } else {
                MSP.send_message(MSPCodes.MSP_SELECT_SETTING, [activeProfile], false, fetch_unique_data);
            }
        }

        // start fetching
        fetch_specific_data_item();
    }

    var uniqueData = [
        MSPCodes.MSP_MISC,
        MSPCodes.MSP_RX_MAP,
        MSPCodes.MSP_BF_CONFIG,
        MSPCodes.MSP_CF_SERIAL_CONFIG,
        MSPCodes.MSP_LED_STRIP_CONFIG,
        MSPCodes.MSP_LED_COLORS
    ];

    function update_unique_data_list() {
        if (semver.gte(CONFIG.apiVersion, "1.8.0")) {
            uniqueData.push(MSPCodes.MSP_LOOP_TIME);
            uniqueData.push(MSPCodes.MSP_ARMING_CONFIG);
        }
        if (semver.gte(CONFIG.apiVersion, "1.14.0")) {
            uniqueData.push(MSPCodes.MSP_3D);
        }
        if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
            uniqueData.push(MSPCodes.MSP_SENSOR_ALIGNMENT);
            uniqueData.push(MSPCodes.MSP_RX_CONFIG);
            uniqueData.push(MSPCodes.MSP_FAILSAFE_CONFIG);
            uniqueData.push(MSPCodes.MSP_RXFAIL_CONFIG);
        }
        if (semver.gte(CONFIG.apiVersion, "1.19.0")) {
            uniqueData.push(MSPCodes.MSP_LED_STRIP_MODECOLOR);
        }
    }
    
    update_unique_data_list();

    function fetch_unique_data() {
        var codeKey = 0;
        
        function fetch_unique_data_item() {
            if (codeKey < uniqueData.length) {
                MSP.send_message(uniqueData[codeKey], false, false, function () {
                    codeKey++;
                    fetch_unique_data_item();
                });
            } else {
                configuration.MISC = jQuery.extend(true, {}, MISC);
                configuration.RCMAP = jQuery.extend(true, [], RC_MAP);
                configuration.BF_CONFIG = jQuery.extend(true, {}, BF_CONFIG);
                configuration.SERIAL_CONFIG = jQuery.extend(true, {}, SERIAL_CONFIG);
                configuration.LED_STRIP = jQuery.extend(true, [], LED_STRIP);
                configuration.LED_COLORS = jQuery.extend(true, [], LED_COLORS);

                if (semver.gte(CONFIG.apiVersion, "1.19.0")) {
                    configuration.LED_MODE_COLORS = jQuery.extend(true, [], LED_MODE_COLORS);
                }
                if (semver.gte(CONFIG.apiVersion, "1.8.0")) {
                    configuration.FC_CONFIG = jQuery.extend(true, {}, FC_CONFIG);
                    configuration.ARMING_CONFIG = jQuery.extend(true, {}, ARMING_CONFIG);
                }
                if (semver.gte(CONFIG.apiVersion, "1.14.0")) {
                    configuration._3D = jQuery.extend(true, {}, _3D);
                }
                if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                    configuration.SENSOR_ALIGNMENT = jQuery.extend(true, {}, SENSOR_ALIGNMENT);
                    configuration.RX_CONFIG = jQuery.extend(true, {}, RX_CONFIG);
                    configuration.FAILSAFE_CONFIG = jQuery.extend(true, {}, FAILSAFE_CONFIG);
                    configuration.RXFAIL_CONFIG = jQuery.extend(true, [], RXFAIL_CONFIG);
                }

                save();
            }
        }
        
        // start fetching
        fetch_unique_data_item();
    }

    function save() {
        var chosenFileEntry = null;

        var accepts = [{
            extensions: ['txt']
        }];

        // generate timestamp for the backup file
        var d = new Date(),
            now = (d.getMonth() + 1) + '.' + d.getDate() + '.' + d.getFullYear() + '.' + d.getHours() + '.' + d.getMinutes();

        // create or load the file
        chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: 'betaflight_backup_' + now, accepts: accepts}, function (fileEntry) {
            if (chrome.runtime.lastError) {
                console.error(chrome.runtime.lastError.message);
                return;
            }

            if (!fileEntry) {
                console.log('No file selected, backup aborted.');
                return;
            }

            chosenFileEntry = fileEntry;

            // echo/console log path specified
            chrome.fileSystem.getDisplayPath(chosenFileEntry, function (path) {
                console.log('Backup file path: ' + path);
            });

            // change file entry from read only to read/write
            chrome.fileSystem.getWritableEntry(chosenFileEntry, function (fileEntryWritable) {
                // check if file is writable
                chrome.fileSystem.isWritableEntry(fileEntryWritable, function (isWritable) {
                    if (isWritable) {
                        chosenFileEntry = fileEntryWritable;

                        // crunch the config object
                        var serialized_config_object = JSON.stringify(configuration);
                        var blob = new Blob([serialized_config_object], {type: 'text/plain'}); // first parameter for Blob needs to be an array

                        chosenFileEntry.createWriter(function (writer) {
                            writer.onerror = function (e) {
                                console.error(e);
                            };

                            var truncated = false;
                            writer.onwriteend = function () {
                                if (!truncated) {
                                    // onwriteend will be fired again when truncation is finished
                                    truncated = true;
                                    writer.truncate(blob.size);

                                    return;
                                }

                                console.log('Write SUCCESSFUL');
                                if (callback) callback();
                            };

                            writer.write(blob);
                        }, function (e) {
                            console.error(e);
                        });
                    } else {
                        // Something went wrong or file is set to read only and cannot be changed
                        console.log('File appears to be read only, sorry.');
                    }
                });
            });
        });
    }

}
    
function configuration_restore(callback) {
    var chosenFileEntry = null;

    var accepts = [{
        extensions: ['txt']
    }];

    // load up the file
    chrome.fileSystem.chooseEntry({type: 'openFile', accepts: accepts}, function (fileEntry) {
        if (chrome.runtime.lastError) {
            console.error(chrome.runtime.lastError.message);
            return;
        }

        if (!fileEntry) {
            console.log('No file selected, restore aborted.');
            return;
        }

        chosenFileEntry = fileEntry;

        // echo/console log path specified
        chrome.fileSystem.getDisplayPath(chosenFileEntry, function (path) {
            console.log('Restore file path: ' + path);
        });

        // read contents into variable
        chosenFileEntry.file(function (file) {
            var reader = new FileReader();

            reader.onprogress = function (e) {
                if (e.total > 1048576) { // 1 MB
                    // dont allow reading files bigger then 1 MB
                    console.log('File limit (1 MB) exceeded, aborting');
                    reader.abort();
                }
            };

            reader.onloadend = function (e) {
                if (e.total != 0 && e.total == e.loaded) {
                    console.log('Read SUCCESSFUL');

                    try { // check if string provided is a valid JSON
                        var configuration = JSON.parse(e.target.result);
                    } catch (e) {
                        // data provided != valid json object
                        console.log('Data provided != valid JSON string, restore aborted.');

                        return;
                    }

                    // validate
                    if (typeof configuration.generatedBy !== 'undefined' && compareVersions(configuration.generatedBy, CONFIGURATOR.backupFileMinVersionAccepted)) {
                                                
                        if (!migrate(configuration)) {
                            GUI.log(chrome.i18n.getMessage('backupFileUnmigratable'));
                            return;
                        }
                        
                        configuration_upload(configuration, callback);
                        
                    } else {
                        GUI.log(chrome.i18n.getMessage('backupFileIncompatible'));
                    }

                    
                }
            };

            reader.readAsText(file);
        });
    });

    function compareVersions(generated, required) {
        if (generated == undefined) {
            return false;
        }
        return semver.gte(generated, required);
    }
    

    function migrate(configuration) {
        var appliedMigrationsCount = 0;
        var migratedVersion = configuration.generatedBy;
        GUI.log(chrome.i18n.getMessage('configMigrationFrom', [migratedVersion]));
        
        if (!compareVersions(migratedVersion, '0.59.1')) {
            
            // variable was renamed
            configuration.MISC.rssi_channel = configuration.MISC.rssi_aux_channel;
            configuration.MISC.rssi_aux_channel = undefined;
            
            migratedVersion = '0.59.1';
            GUI.log(chrome.i18n.getMessage('configMigratedTo', [migratedVersion]));
            appliedMigrationsCount++;
        }
        
        if (!compareVersions(migratedVersion, '0.60.1')) {
            
            // LED_STRIP support was added.
            if (!configuration.LED_STRIP) {
                configuration.LED_STRIP = [];
            }
            
            migratedVersion = '0.60.1';
            GUI.log(chrome.i18n.getMessage('configMigratedTo', [migratedVersion]));
            appliedMigrationsCount++;
        }
        
        if (!compareVersions(migratedVersion, '0.61.0')) {
            
            // Changing PID controller via UI was added.
            if (!configuration.PIDs && configuration.PID) {
                configuration.PIDs = configuration.PID;
                configuration.PID = {
                    controller: 0 // assume pid controller 0 was used.
                };
            }
            
            migratedVersion = '0.61.0';
            GUI.log(chrome.i18n.getMessage('configMigratedTo', [migratedVersion]));
            appliedMigrationsCount++;
        }

        if (!compareVersions(migratedVersion, '0.63.0')) {
            
            // LED Strip was saved as object instead of array.
            if (typeof(configuration.LED_STRIP) == 'object') {
                var fixed_led_strip = [];

                var index = 0;
                while (configuration.LED_STRIP[index]) {
                    fixed_led_strip.push(configuration.LED_STRIP[index++]);
                }
                configuration.LED_STRIP = fixed_led_strip;
            }

            
            for (var profileIndex = 0; profileIndex < 3; profileIndex++) {
                var RC = configuration.profiles[profileIndex].RC;
                // TPA breakpoint was added
                if (!RC.dynamic_THR_breakpoint) {
                    RC.dynamic_THR_breakpoint = 1500; // firmware default
                }
                
                // Roll and pitch rates were split
                RC.roll_rate = RC.roll_pitch_rate;
                RC.pitch_rate = RC.roll_pitch_rate;
            }
            
            migratedVersion = '0.63.0';
            GUI.log(chrome.i18n.getMessage('configMigratedTo', [migratedVersion]));
            appliedMigrationsCount++;
        }

        if (configuration.apiVersion == undefined) {
            configuration.apiVersion = "1.0.0" // a guess that will satisfy the rest of the code
        }
        // apiVersion previously stored without patchlevel
        if (!semver.parse(configuration.apiVersion)) {
            configuration.apiVersion += ".0";
            if (!semver.parse(configuration.apiVersion)) {
                return false;
            }
        }
        if (compareVersions(migratedVersion, '0.63.0') && !compareVersions(configuration.apiVersion, '1.7.0')) {
            // Serial configuation redesigned, 0.63.0 saves old and new configurations.
            var ports = [];
            for (var portIndex = 0; portIndex < configuration.SERIAL_CONFIG.ports.length; portIndex++) {
                var oldPort = configuration.SERIAL_CONFIG.ports[portIndex];

                var newPort = {
                    identifier: oldPort.identifier,
                    functions: [],
                    msp_baudrate: String(configuration.SERIAL_CONFIG.mspBaudRate),
                    gps_baudrate: String(configuration.SERIAL_CONFIG.gpsBaudRate),
                    telemetry_baudrate: 'AUTO',
                    blackbox_baudrate: '115200',
                };
                
                switch(oldPort.scenario) {
                    case 1: // MSP, CLI, TELEMETRY, SMARTPORT TELEMETRY, GPS-PASSTHROUGH
                    case 5: // MSP, CLI, GPS-PASSTHROUGH
                    case 8: // MSP ONLY
                        newPort.functions.push('MSP');
                    break;
                    case 2: // GPS 
                        newPort.functions.push('GPS');
                    break;
                    case 3: // RX_SERIAL 
                        newPort.functions.push('RX_SERIAL');
                    break;
                    case 10: // BLACKBOX ONLY
                        newPort.functions.push('BLACKBOX');
                    break;
                    case 11: // MSP, CLI, BLACKBOX, GPS-PASSTHROUGH
                        newPort.functions.push('MSP');
                        newPort.functions.push('BLACKBOX');
                    break;
                }
                
                ports.push(newPort);
            }
            configuration.SERIAL_CONFIG = { 
                ports: ports 
            };
            
            appliedMigrationsCount++;
        }
        
        if (compareVersions(migratedVersion, '0.63.0') && !compareVersions(configuration.apiVersion, '1.8.0')) {
            // api 1.8 exposes looptime and arming config
            
            if (configuration.FC_CONFIG == undefined) {
                configuration.FC_CONFIG = {
                    loopTime: 3500
                };
            }

            if (configuration.ARMING_CONFIG == undefined) {
                configuration.ARMING_CONFIG = {
                    auto_disarm_delay:      5,
                    disarm_kill_switch:     1
                };
            }
            appliedMigrationsCount++;
        }
        
        if (compareVersions(migratedVersion, '0.63.0')) {
            // backups created with 0.63.0 for firmwares with api < 1.8 were saved with incorrect looptime
            if (configuration.FC_CONFIG.loopTime == 0) {
                //reset it to the default
                configuration.FC_CONFIG.loopTime = 3500;
            }
        }
        
        if (semver.lt(migratedVersion, '0.66.0')) {
            // api 1.12 updated servo configuration protocol and added servo mixer rules
            for (var profileIndex = 0; profileIndex < configuration.profiles.length; profileIndex++) {
                
                if (semver.eq(configuration.apiVersion, '1.10.0')) {
                    // drop two unused servo configurations
                    while (configuration.profiles[profileIndex].ServoConfig.length > 8) {
                        configuration.profiles[profileIndex].ServoConfig.pop();
                    } 
                }
                
                for (var i = 0; i < configuration.profiles[profileIndex].ServoConfig.length; i++) {
                    var servoConfig = profiles[profileIndex].ServoConfig;
                    
                    servoConfig[i].angleAtMin = 45;
                    servoConfig[i].angleAtMax = 45;
                    servoConfig[i].reversedInputSources = 0;
                    
                    // set the rate to 0 if an invalid value is detected.
                    if (servoConfig[i].rate < -100 || servoConfig[i].rate > 100) {
                        servoConfig[i].rate = 0;
                    }
                }

                configuration.profiles[profileIndex].ServoRules = [];
            }
            
            migratedVersion = '0.66.0';

            appliedMigrationsCount++;
        }
        
        if (semver.lt(configuration.apiVersion, '1.14.0') && semver.gte(CONFIG.apiVersion, "1.14.0")) {
            // api 1.14 removed old pid controllers
            for (var profileIndex = 0; profileIndex < configuration.profiles.length; profileIndex++) {
                var newPidControllerIndex = configuration.profiles[profileIndex].PID.controller;
                switch (newPidControllerIndex) {
                    case 3: 
                    case 4: 
                    case 5: 
                        newPidControllerIndex = 0;
                        break;
                }
                configuration.profiles[profileIndex].PID.controller = newPidControllerIndex;
            }
            appliedMigrationsCount++;
        }
        

        if (compareVersions(migratedVersion, '0.66.0') && !compareVersions(configuration.apiVersion, '1.14.0')) {
            // api 1.14 exposes 3D configuration
            
            if (configuration._3D == undefined) {
                configuration._3D = {
                    deadband3d_low:         1406,
                    deadband3d_high:        1514,
                    neutral3d:              1460,
                    deadband3d_throttle:    50
                };
            }
            appliedMigrationsCount++;
        }
        
        
        if (compareVersions(migratedVersion, '0.66.0') && !compareVersions(configuration.apiVersion, '1.15.0')) {
            // api 1.15 exposes RCdeadband and sensor alignment

            
            for (var profileIndex = 0; profileIndex < configuration.profiles.length; profileIndex++) {
                 if (configuration.profiles[profileIndex].RCdeadband == undefined) {
                    configuration.profiles[profileIndex].RCdeadband = {
                    deadband:                0,
                    yaw_deadband:            0,
                    alt_hold_deadband:       40,
                    };                
                }
            }
            if (configuration.SENSOR_ALIGNMENT == undefined) {
                    configuration.SENSOR_ALIGNMENT = {
                    align_gyro:              0,
                    align_acc:               0,
                    align_mag:               0
                    };                
            }
        
            // api 1.15 exposes RX_CONFIG, FAILSAFE_CONFIG and RXFAIL_CONFIG configuration

            if (configuration.RX_CONFIG == undefined) {
                configuration.RX_CONFIG = {
                    serialrx_provider:      0,
                    spektrum_sat_bind:      0,
                    midrc:                  1500,
                    mincheck:               1100,
                    maxcheck:               1900,
                    rx_min_usec:            885,
                    rx_max_usec:            2115
                };
            }

            if (configuration.FAILSAFE_CONFIG == undefined) {
                configuration.FAILSAFE_CONFIG = {
                    failsafe_delay:                 10,
                    failsafe_off_delay:             200,
                    failsafe_throttle:              1000,
                    failsafe_kill_switch:           0,
                    failsafe_throttle_low_delay:    100,
                    failsafe_procedure:             0
                };
            }

            if (configuration.RXFAIL_CONFIG == undefined) {
                configuration.RXFAIL_CONFIG = [
                    {mode: 0, value: 1500},
                    {mode: 0, value: 1500},
                    {mode: 0, value: 1500},
                    {mode: 0, value: 875}
                ];

                for (var i = 0; i < 14; i++) {
                    var rxfailChannel = {
                        mode:  1,
                        value: 1500
                    };
                    configuration.RXFAIL_CONFIG.push(rxfailChannel);
                }
            }

            appliedMigrationsCount++;
        }

        if (compareVersions(migratedVersion, '1.2.0')) {
            // old version of the configurator incorrectly had a 'disabled' option for GPS SBAS mode.
            if (MISC.gps_ubx_sbas < 0) {
                MISC.gps_ubx_sbas = 0;
            }
            migratedVersion = '1.2.0';

            appliedMigrationsCount++;
        }

        if (!compareVersions(migratedVersion, '1.3.1')) {
            
            // LED_COLORS & LED_MODE_COLORS support was added.
            if (!configuration.LED_COLORS) {
                configuration.LED_COLORS = [];
            }
            if (!configuration.LED_MODE_COLORS) {
                configuration.LED_MODE_COLORS = [];
            }

            migratedVersion = '1.3.1';
            GUI.log(chrome.i18n.getMessage('configMigratedTo', [migratedVersion]));
            appliedMigrationsCount++;
        }
        
        if (appliedMigrationsCount > 0) {
            GUI.log(chrome.i18n.getMessage('configMigrationSuccessful', [appliedMigrationsCount]));
        }        
        return true;
    }
    
    function configuration_upload(configuration, callback) {
        function upload() {
            var activeProfile = null;
            var numProfiles = CONFIG.numProfiles;
            if (configuration.profiles.length < numProfiles) {
                numProfiles = configuration.profiles.length;
            }

            var profileSpecificData = [
                MSPCodes.MSP_SET_PID_CONTROLLER,
                MSPCodes.MSP_SET_PID,
                MSPCodes.MSP_SET_RC_TUNING,
                MSPCodes.MSP_SET_ACC_TRIM
            ];

            if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                profileSpecificData.push(MSPCodes.MSP_SET_RC_DEADBAND);
            }

            MSP.send_message(MSPCodes.MSP_STATUS, false, false, function () {
                activeProfile = CONFIG.profile;
                select_profile();
            });

            function select_profile() {
                if (activeProfile > 0) {
                    MSP.send_message(MSPCodes.MSP_SELECT_SETTING, [0], false, upload_specific_data);
                } else {
                    upload_specific_data();
                }
            }

            function upload_specific_data() {
                var savingProfile = 0,
                    codeKey = 0;

                function load_objects(profile) {
                    PID = configuration.profiles[profile].PID;
                    PIDs = configuration.profiles[profile].PIDs;
                    RC_tuning = configuration.profiles[profile].RC;
                    CONFIG.accelerometerTrims = configuration.profiles[profile].AccTrim;
                    SERVO_CONFIG = configuration.profiles[profile].ServoConfig;
                    SERVO_RULES = configuration.profiles[profile].ServoRules;
                    MODE_RANGES = configuration.profiles[profile].ModeRanges;
                    ADJUSTMENT_RANGES = configuration.profiles[profile].AdjustmentRanges;
                    RC_deadband = configuration.profiles[profile].RCdeadband;
                }

                function upload_using_specific_commands() {
                    MSP.send_message(profileSpecificData[codeKey], mspHelper.crunch(profileSpecificData[codeKey]), false, function () {
                        codeKey++;

                        if (codeKey < profileSpecificData.length) {
                            upload_using_specific_commands();
                        } else {
                            codeKey = 0;
                            savingProfile++;

                            if (savingProfile < numProfiles) {
                                load_objects(savingProfile);

                                MSP.send_message(MSPCodes.MSP_EEPROM_WRITE, false, false, function () {
                                    MSP.send_message(MSPCodes.MSP_SELECT_SETTING, [savingProfile], false, upload_using_specific_commands);
                                });
                            } else {
                                MSP.send_message(MSPCodes.MSP_EEPROM_WRITE, false, false, function () {
                                    MSP.send_message(MSPCodes.MSP_SELECT_SETTING, [activeProfile], false, upload_unique_data);
                                });
                            }
                        }
                    });
                }

                function upload_servo_configuration() {
                    mspHelper.sendServoConfigurations(upload_mode_ranges);
                }

                function upload_mode_ranges() {
                    mspHelper.sendModeRanges(upload_adjustment_ranges);
                }
                
                function upload_adjustment_ranges() {
                    mspHelper.sendAdjustmentRanges(upload_using_specific_commands);
                }
                // start uploading
                load_objects(0);
                upload_servo_configuration();
            }

            function upload_unique_data() {
                var codeKey = 0;

                var uniqueData = [
                    MSPCodes.MSP_SET_MISC,
                    MSPCodes.MSP_SET_RX_MAP,
                    MSPCodes.MSP_SET_BF_CONFIG,
                    MSPCodes.MSP_SET_CF_SERIAL_CONFIG
                ];
                
                function update_unique_data_list() {
                    if (semver.gte(CONFIG.apiVersion, "1.8.0")) {
                        uniqueData.push(MSPCodes.MSP_SET_LOOP_TIME);
                        uniqueData.push(MSPCodes.MSP_SET_ARMING_CONFIG);
                    }
                    if (semver.gte(CONFIG.apiVersion, "1.14.0")) {
                        uniqueData.push(MSPCodes.MSP_SET_3D);
                    }
                    if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                        uniqueData.push(MSPCodes.MSP_SET_SENSOR_ALIGNMENT);
                        uniqueData.push(MSPCodes.MSP_SET_RX_CONFIG);
                        uniqueData.push(MSPCodes.MSP_SET_FAILSAFE_CONFIG);
                    }
                }
                
                function load_objects() {
                    MISC = configuration.MISC;
                    RC_MAP = configuration.RCMAP;
                    BF_CONFIG = configuration.BF_CONFIG;
                    SERIAL_CONFIG = configuration.SERIAL_CONFIG;
                    LED_STRIP = configuration.LED_STRIP;
                    LED_COLORS = configuration.LED_COLORS;
                    LED_MODE_COLORS = configuration.LED_MODE_COLORS;
                    ARMING_CONFIG = configuration.ARMING_CONFIG;
                    FC_CONFIG = configuration.FC_CONFIG;
                    _3D = configuration._3D;
                    SENSOR_ALIGNMENT = configuration.SENSOR_ALIGNMENT;
                    RX_CONFIG = configuration.RX_CONFIG;
                    FAILSAFE_CONFIG = configuration.FAILSAFE_CONFIG;
                    RXFAIL_CONFIG = configuration.RXFAIL_CONFIG;
                }

                function send_unique_data_item() {
                    if (codeKey < uniqueData.length) {
                        MSP.send_message(uniqueData[codeKey], mspHelper.crunch(uniqueData[codeKey]), false, function () {
                            codeKey++;
                            send_unique_data_item();
                        });
                    } else {
                        send_led_strip_config();
                    }
                }

                load_objects();

                update_unique_data_list();

                // start uploading
                send_unique_data_item();
            }

            function send_led_strip_config() {
                mspHelper.sendLedStripConfig(send_led_strip_colors);
            }

            function send_led_strip_colors() {
                mspHelper.sendLedStripColors(send_led_strip_mode_colors);
            }

            function send_led_strip_mode_colors() {
                if (semver.gte(CONFIG.apiVersion, "1.19.0"))
                    mspHelper.sendLedStripModeColors(send_rxfail_config);
                else
                    send_rxfail_config();
            }
            
            function send_rxfail_config() {
                if (semver.gte(CONFIG.apiVersion, "1.15.0")) {
                    mspHelper.sendRxFailConfig(save_to_eeprom);
                } else {
                    save_to_eeprom();
                }
            }

            function save_to_eeprom() {
                MSP.send_message(MSPCodes.MSP_EEPROM_WRITE, false, false, reboot);
            }

            function reboot() {
                GUI.log(chrome.i18n.getMessage('eeprom_saved_ok'));

                GUI.tab_switch_cleanup(function() {
                    MSP.send_message(MSPCodes.MSP_SET_REBOOT, false, false, reinitialize);
                });
            }

            function reinitialize() {
                GUI.log(chrome.i18n.getMessage('deviceRebooting'));

                GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                    MSP.send_message(MSPCodes.MSP_STATUS, false, false, function() {
                        GUI.log(chrome.i18n.getMessage('deviceReady'));
                        if (callback) callback();
                    });
                }, 1500); // 1500 ms seems to be just the right amount of delay to prevent data request timeouts
            }
        }

        upload();
    }
}
