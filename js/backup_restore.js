'use strict';

// code below is highly experimental, although it runs fine on latest firmware
// the data inside nested objects needs to be verified if deep copy works properly
function configuration_backup(callback) {
    var activeProfile = null,
        profilesN = 3;

    var profileSpecificData = [
        MSP_codes.MSP_PID,
        MSP_codes.MSP_RC_TUNING,
        MSP_codes.MSP_ACC_TRIM,
        MSP_codes.MSP_SERVO_CONF
    ];

    var uniqueData = [
        MSP_codes.MSP_BOX,
        MSP_codes.MSP_MISC,
        MSP_codes.MSP_RCMAP,
        MSP_codes.MSP_CONFIG
    ];

    var configuration = {
        'generatedBy': chrome.runtime.getManifest().version,
        'profiles': []
    };

    MSP.send_message(MSP_codes.MSP_STATUS, false, false, function () {
        activeProfile = CONFIG.profile;
        select_profile();
    });

    function select_profile() {
        if (activeProfile > 0) {
            MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [0], false, fetch_specific_data);
        } else {
            fetch_specific_data();
        }
    }

    function fetch_specific_data() {
        var fetchingProfile = 0,
            codeKey = 0;

        function query() {
            if (fetchingProfile < profilesN) {
                MSP.send_message(profileSpecificData[codeKey], false, false, function () {
                    codeKey++;

                    if (codeKey < profileSpecificData.length) {
                        query();
                    } else {
                        configuration.profiles.push({
                            'PID': jQuery.extend(true, [], PIDs),
                            'RC': jQuery.extend(true, {}, RC_tuning),
                            'AccTrim': jQuery.extend(true, [], CONFIG.accelerometerTrims),
                            'ServoConfig': jQuery.extend(true, [], SERVO_CONFIG)
                        });

                        codeKey = 0;
                        fetchingProfile++;

                        MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [fetchingProfile], false, query);
                    }
                });
            } else {
                MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [activeProfile], false, fetch_unique_data);
            }
        }

        // start fetching
        query();
    }

    function fetch_unique_data() {
        var codeKey = 0;

        function query() {
            if (codeKey < uniqueData.length) {
                MSP.send_message(uniqueData[codeKey], false, false, function () {
                    codeKey++;
                    query();
                });
            } else {
                configuration.AUX = jQuery.extend(true, [], AUX_CONFIG_values);
                configuration.MISC = jQuery.extend(true, {}, MISC);
                configuration.RCMAP = jQuery.extend(true, [], RC_MAP);
                configuration.BF_CONFIG = jQuery.extend(true, {}, BF_CONFIG);

                save();
            }
        }

        // start fetching
        query();
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
        chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: 'baseflight_backup_' + now, accepts: accepts}, function (fileEntry) {
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
                        var deserialized_configuration_object = JSON.parse(e.target.result);
                    } catch (e) {
                        // data provided != valid json object
                        console.log('Data provided != valid JSON string, restore aborted.');

                        return;
                    }

                    configuration_upload(deserialized_configuration_object, callback);
                }
            };

            reader.readAsText(file);
        });
    });

    function configuration_upload(configuration, callback) {
        function compareVersions(generated, required) {
            var a = generated.split('.'),
                b = required.split('.');

            for (var i = 0; i < a.length; ++i) {
                a[i] = Number(a[i]);
            }
            for (var i = 0; i < b.length; ++i) {
                b[i] = Number(b[i]);
            }
            if (a.length == 2) {
                a[2] = 0;
            }

            if (a[0] > b[0]) return true;
            if (a[0] < b[0]) return false;

            if (a[1] > b[1]) return true;
            if (a[1] < b[1]) return false;

            if (a[2] > b[2]) return true;
            if (a[2] < b[2]) return false;

            return true;
        }

        function upload() {
            var activeProfile = null,
                profilesN = 3;

            var profileSpecificData = [
                MSP_codes.MSP_SET_PID,
                MSP_codes.MSP_SET_RC_TUNING,
                MSP_codes.MSP_SET_ACC_TRIM,
                MSP_codes.MSP_SET_SERVO_CONF
            ];

            var uniqueData = [
                MSP_codes.MSP_SET_BOX,
                MSP_codes.MSP_SET_MISC,
                MSP_codes.MSP_SET_RCMAP,
                MSP_codes.MSP_SET_CONFIG
            ];

            MSP.send_message(MSP_codes.MSP_STATUS, false, false, function () {
                activeProfile = CONFIG.profile;
                select_profile();
            });

            function select_profile() {
                if (activeProfile > 0) {
                    MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [0], false, upload_specific_data);
                } else {
                    upload_specific_data();
                }
            }

            function upload_specific_data() {
                var savingProfile = 0,
                    codeKey = 0;

                function load_objects(profile) {
                    PIDs = configuration.profiles[profile].PID;
                    RC_tuning = configuration.profiles[profile].RC;
                    CONFIG.accelerometerTrims = configuration.profiles[profile].AccTrim;
                    SERVO_CONFIG = configuration.profiles[profile].ServoConfig;
                }

                function query() {
                    MSP.send_message(profileSpecificData[codeKey], MSP.crunch(profileSpecificData[codeKey]), false, function () {
                        codeKey++;

                        if (codeKey < profileSpecificData.length) {
                            query();
                        } else {
                            codeKey = 0;
                            savingProfile++;

                            if (savingProfile < profilesN) {
                                load_objects(savingProfile);

                                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                                    MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [savingProfile], false, query);
                                });
                            } else {
                                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                                    MSP.send_message(MSP_codes.MSP_SELECT_SETTING, [activeProfile], false, upload_unique_data);
                                });
                            }
                        }
                    });
                }

                // start uploading
                load_objects(0);
                query();
            }

            function upload_unique_data() {
                var codeKey = 0;

                function load_objects() {
                    AUX_CONFIG_values = configuration.AUX;
                    MISC = configuration.MISC;
                    RC_MAP = configuration.RCMAP;
                    BF_CONFIG = configuration.BF_CONFIG;
                }

                function query() {
                    if (codeKey < uniqueData.length) {
                        MSP.send_message(uniqueData[codeKey], MSP.crunch(uniqueData[codeKey]), false, function () {
                            codeKey++;
                            query();
                        });
                    } else {
                        MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, reboot);
                    }
                }

                // start uploading
                load_objects();
                query();
            }

            function reboot() {
                GUI.log(chrome.i18n.getMessage('eeprom_saved_ok'));

                GUI.tab_switch_cleanup(function() {
                    MSP.send_message(MSP_codes.MSP_SET_REBOOT, false, false, reinitialize);
                });
            }

            function reinitialize() {
                GUI.log(chrome.i18n.getMessage('deviceRebooting'));

                GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                    MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                        GUI.log(chrome.i18n.getMessage('deviceReady'));

                        if (callback) callback();
                    });
                }, 1500); // 1500 ms seems to be just the right amount of delay to prevent data request timeouts
            }
        }

        // validate
        if (typeof configuration.generatedBy !== 'undefined' && compareVersions(configuration.generatedBy, CONFIGURATOR.backupFileMinVersionAccepted)) {
            upload();
        } else {
            GUI.log(chrome.i18n.getMessage('backupFileIncompatible'));
        }
    }
}