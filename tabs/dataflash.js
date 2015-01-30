'use strict';

TABS.dataflash = {};
TABS.dataflash.initialize = function (callback) {
    var self = this;

    if (GUI.active_tab != 'dataflash') {
        GUI.active_tab = 'dataflash';
        googleAnalytics.sendAppView('dataflash');
    }

    var requested_properties = [],
        samples = 0,
        requests = 0,
        log_buffer = [];

    if (CONFIGURATOR.connectionValid) {
        MSP.send_message(MSP_codes.MSP_DATAFLASH_SUMMARY, false, false, function() {
            $('#content').load("./tabs/dataflash.html", process_html);
        });
    }
    
    function process_html() {
        // translate to user-selected language
        localize();

        $(".tab-dataflash .dataflash-capacity").text(DATAFLASH.totalSize);
        $(".tab-dataflash .dataflash-sectors").text(DATAFLASH.sectors);
        
        // UI hooks
        $('.tab-dataflash a.erase_flash').click(erase_flash);

        $('.tab-dataflash a.save_to_file').click(stream_flash_to_file);

        if (callback) callback();
    }

    // IO related methods
    function zeroPad(value, width) {
        value = "" + value;
        
        while (value.length < width) {
            value = "0" + value;
        }
        
        return value;
    }
    
    function stream_flash_to_file() {
        if (GUI.connected_to) {
            prepare_file(function(fileWriter) {
                var
                    nextAddress = 0;
                
                function onChunkRead(chunkAddress, chunkDataView) {
                    // If we didn't get a zero-byte chunk (indicating end-of-file), request more
                    if (chunkDataView.byteLength > 0) {
                        var blob = new Blob([chunkDataView]);
                        
                        fileWriter.write(blob);
                        
                        nextAddress += chunkDataView.byteLength;
                        MSP.dataflashRead(nextAddress, onChunkRead);
                    }
                }
                
                MSP.dataflashRead(nextAddress, onChunkRead);
            });
        }
    }
    
    function prepare_file(onComplete) {
        var 
            date = new Date(),
            filename = 'blackbox_log_' + date.getFullYear() + '-'  + zeroPad(date.getMonth() + 1, 2) + '-' 
                + zeroPad(date.getDate(), 2) + '_' + zeroPad(date.getHours(), 2) + zeroPad(date.getMinutes(), 2) 
                + zeroPad(date.getSeconds(), 2);
        
        chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: filename, 
                accepts: [{extensions: ['TXT']}]}, function(fileEntry) {
            if (!fileEntry) {
                console.log('No file selected');
                return;
            }

            // echo/console log path specified
            chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                console.log('Dataflash dump file path: ' + path);
            });

            prepare_writer(fileEntry, onComplete);
        });
    }

    function prepare_writer(fileEntry, onComplete) {
        fileEntry.createWriter(function (fileWriter) {
            fileWriter.onerror = function (e) {
                console.error(e);

                // stop logging if the procedure was/is still running
            };

            fileWriter.onwriteend = function () {
            };

            onComplete(fileWriter);
        }, function (e) {
            // File is not readable or does not exist!
            console.error(e);
        });
    }
    
    function erase_flash() {
        /* var dialog = $("<dialog>lol</dialog>");
        
        $("body").append(dialog);
        
        dialog[0].showModal(); 
        
        TODO modal dialog to confirm erase */
        
        MSP.send_message(MSP_codes.MSP_DATAFLASH_ERASE, false, false, function(data) {
            
        });
    }
};

TABS.dataflash.cleanup = function (callback) {
    if (callback) callback();
};