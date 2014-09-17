'use strict';

function configuration_backup() {
    // request configuration data (one by one)

    function get_ident_data() {
        MSP.send_message(MSP_codes.MSP_IDENT, false, false, get_status_data);
    }

    function get_status_data() {
        MSP.send_message(MSP_codes.MSP_STATUS, false, false, get_pid_data);
    }

    function get_pid_data() {
        MSP.send_message(MSP_codes.MSP_PID, false, false, get_rc_tuning_data);
    }

    function get_rc_tuning_data() {
        MSP.send_message(MSP_codes.MSP_RC_TUNING, false, false, get_box_names_data);
    }

    function get_box_names_data() {
        MSP.send_message(MSP_codes.MSP_BOXNAMES, false, false, get_box_data);
    }

    function get_box_data() {
        MSP.send_message(MSP_codes.MSP_BOX, false, false, get_acc_trim_data);
    }

    function get_acc_trim_data() {
        MSP.send_message(MSP_codes.MSP_ACC_TRIM, false, false, get_misc_data);
    }

    function get_misc_data() {
        MSP.send_message(MSP_codes.MSP_MISC, false, false, backup);
    }

    function backup() {
        var chosenFileEntry = null;

        var accepts = [{
            extensions: ['txt']
        }];

        // generate timestamp for the backup file
        var d = new Date(),
            now = d.getUTCFullYear() + '.' + d.getDate() + '.' + (d.getMonth() + 1) + '.' + d.getHours() + '.' + d.getMinutes();

        // create or load the file
        chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: 'baseflight_backup_' + now, accepts: accepts}, function (fileEntry) {
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

                        // create config object that will be used to store all downloaded data
                        var configuration = {
                            'firmware_version': CONFIG.version,
                            'configurator_version': chrome.runtime.getManifest().version,
                            'PID': PIDs,
                            'AUX_val': AUX_CONFIG_values,
                            'RC': RC_tuning,
                            'AccelTrim': CONFIG.accelerometerTrims,
                            'MISC': MISC
                        };

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

    // begin fetching latest data
    get_ident_data();
}

function configuration_restore() {
    var chosenFileEntry = null;

    var accepts = [{
        extensions: ['txt']
    }];

    // load up the file
    chrome.fileSystem.chooseEntry({type: 'openFile', accepts: accepts}, function (fileEntry) {
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

                    // replacing "old configuration" with configuration from backup file
                    var configuration = deserialized_configuration_object;

                    // some configuration.VERSION code goes here? will see

                    PIDs = configuration.PID;
                    AUX_CONFIG_values = configuration.AUX_val;
                    RC_tuning = configuration.RC;
                    CONFIG.accelerometerTrims = configuration.AccelTrim;
                    MISC = configuration.MISC;

                    // all of the arrays/objects are set, upload changes
                    configuration_upload();
                }
            };

            reader.readAsText(file);
        });
    });
}

function configuration_upload() {
    function rc_tuning() { // Send over the RC_tuning changes
        MSP.send_message(MSP_codes.MSP_SET_RC_TUNING, MSP.crunch(MSP_codes.MSP_SET_RC_TUNING), false, aux);
    }

    function aux() { // Send over the AUX changes
        MSP.send_message(MSP_codes.MSP_SET_BOX, MSP.crunch(MSP_codes.MSP_SET_BOX), false, trim);
    }

    function trim() { // Send over the new trims
        MSP.send_message(MSP_codes.MSP_SET_ACC_TRIM, MSP.crunch(MSP_codes.MSP_SET_ACC_TRIM), false, misc);
    }

    function misc() { // Send ove the new MISC
        MSP.send_message(MSP_codes.MSP_SET_MISC, MSP.crunch(MSP_codes.MSP_SET_MISC), false, save_eeprom);
    }

    function save_eeprom() {
        MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
            GUI.log(chrome.i18n.getMessage('eeprom_saved_ok'));
        });
    }

    // Send over the PID changes
    MSP.send_message(MSP_codes.MSP_SET_PID, MSP.crunch(MSP_codes.MSP_SET_PID), false, rc_tuning);
}