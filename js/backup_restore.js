function configuration_backup() {
    // request configuration data
    send_message(MSP_codes.MSP_IDENT, MSP_codes.MSP_IDENT);
    send_message(MSP_codes.MSP_STATUS, MSP_codes.MSP_STATUS);
    send_message(MSP_codes.MSP_PID, MSP_codes.MSP_PID);
    send_message(MSP_codes.MSP_RC_TUNING, MSP_codes.MSP_RC_TUNING);
    send_message(MSP_codes.MSP_BOXNAMES, MSP_codes.MSP_BOXNAMES);
    send_message(MSP_codes.MSP_BOX, MSP_codes.MSP_BOX);
    send_message(MSP_codes.MSP_ACC_TRIM, MSP_codes.MSP_ACC_TRIM);
    
    // applying 200ms delay (should be enough to pull all the data down)
    // we might increase this in case someone would be using very slow baudrate (ergo 9600 and lower)
    setTimeout(function() {
        var chosenFileEntry = null;
        
        var accepts = [{
            extensions: ['txt']
        }];
        
        // generate timestamp for the backup file
        var d = new Date();
        var now = d.getUTCFullYear() + '.' + d.getDate() + '.' + (d.getMonth() + 1) + '.' + d.getHours() + '.' + d.getMinutes();  

        // create or load the file
        chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: 'bf_mw_backup_' + now, accepts: accepts}, function(fileEntry) {
            if (!fileEntry) {
                console.log('No file selected');
                
                return;
            }
            
            chosenFileEntry = fileEntry; 

            // echo/console log path specified
            chrome.fileSystem.getDisplayPath(chosenFileEntry, function(path) {
                console.log('Backup file path: ' + path);
            });

            // change file entry from read only to read/write
            chrome.fileSystem.getWritableEntry(chosenFileEntry, function(fileEntryWritable) {
                // check if file is writable
                chrome.fileSystem.isWritableEntry(fileEntryWritable, function(isWritable) {
                    if (isWritable) {
                        chosenFileEntry = fileEntryWritable;
                    
                        // create config object that will be used to store all downloaded data
                        var configuration = {
                            PID: PIDs,
                            AUX_val: AUX_CONFIG_values,
                            RC: RC_tuning,
                            AccelTrim: CONFIG.accelerometerTrims
                        }
                        
                        // crunch the config object
                        var serialized_config_object = JSON.stringify(configuration);
                        var blob = new Blob([serialized_config_object], {type: 'text/plain'}); // first parameter for Blob needs to be an array
                        
                        chosenFileEntry.createWriter(function(writer) {
                            writer.onerror = function (e) {
                                console.error(e);
                            };
                            
                            writer.onwriteend = function() {
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
    }, 200);
}

function configuration_restore() {
    var chosenFileEntry = null;
    
    var accepts = [{
        extensions: ['txt']
    }];
    
    // load up the file
    chrome.fileSystem.chooseEntry({type: 'openFile', accepts: accepts}, function(fileEntry) {
        if (!fileEntry) {
            console.log('No file selected');
            
            return;
        }
        
        chosenFileEntry = fileEntry; 
        
        // echo/console log path specified
        chrome.fileSystem.getDisplayPath(chosenFileEntry, function(path) {
            console.log('Restore file path: ' + path);
        }); 

        // read contents into variable
        chosenFileEntry.file(function(file) {
            var reader = new FileReader();

            reader.onerror = function (e) {
                console.error(e);
            };
            
            reader.onloadend = function(e) {
                console.log('Read SUCCESSFUL');
                
                try { // check if string provided is a valid JSON
                    var deserialized_configuration_object = JSON.parse(e.target.result);
                } catch (e) {
                    // data provided != valid json object
                    console.log('Data provided != valid JSON string. ABORTING !!!');
                    
                    return;
                }
                
                // replacing "old configuration" with configuration from backup file
                var configuration = deserialized_configuration_object;
                console.log(configuration);
            };

            reader.readAsText(file);
        });
    });
}