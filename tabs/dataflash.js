'use strict';

TABS.dataflash = {
    available: false
};
TABS.dataflash.initialize = function (callback) {
    var 
        self = this,
        saveCancelled, eraseCancelled;

    if (GUI.active_tab != 'dataflash') {
        GUI.active_tab = 'dataflash';
        googleAnalytics.sendAppView('dataflash');
    }

    var 
        requested_properties = [],
        samples = 0,
        requests = 0,
        log_buffer = [];

    if (CONFIGURATOR.connectionValid) {
        TABS.dataflash.available = (CONFIG.apiVersion >= 1.6)
        
        if (!TABS.dataflash.available) {
            load_html();
            return;
        }
        
        MSP.send_message(MSP_codes.MSP_DATAFLASH_SUMMARY, false, false, load_html);
    }
    
    function load_html() {
        $('#content').load("./tabs/dataflash.html", function() {
            create_html();
        });
    }
    
    function formatFilesize(bytes) {
        if (bytes < 1024) {
            return bytes + "B";
        }
        
        var kilobytes = bytes / 1024;
        
        if (kilobytes < 1024) {
            return Math.round(kilobytes) + "kB";
        }
        
        var megabytes = kilobytes / 1024;
        
        return megabytes.toFixed(1) + "MB";
    }
    
    function update_html() {
        if (DATAFLASH.usedSize > 0) {
            $(".tab-dataflash .dataflash-used").css({
                width: (DATAFLASH.usedSize / DATAFLASH.totalSize * 100) + "%",
                display: 'block'
            });
            
            $(".tab-dataflash .dataflash-used div").text('Used space ' + formatFilesize(DATAFLASH.usedSize));
        } else {
            $(".tab-dataflash .dataflash-used").css({
                display: 'none'
            });
        }

        if (DATAFLASH.totalSize - DATAFLASH.usedSize > 0) {
            $(".tab-dataflash .dataflash-free").css({
                width: ((DATAFLASH.totalSize - DATAFLASH.usedSize) / DATAFLASH.totalSize * 100) + "%",
                display: 'block'
            });
            $(".tab-dataflash .dataflash-free div").text('Free space ' + formatFilesize(DATAFLASH.totalSize - DATAFLASH.usedSize));
        } else {
            $(".tab-dataflash .dataflash-free").css({
                display: 'none'
            });
        }
        
        $(".tab-dataflash a.erase-flash, .tab-dataflash a.save-flash").toggleClass("disabled", DATAFLASH.usedSize == 0);
    }
    
    function create_html() {
        
        // translate to user-selected language
        localize();
       
        if (TABS.dataflash.available) {
            var supportsDataflash = DATAFLASH.totalSize > 0;
            
            $(".tab-dataflash").toggleClass("supported", supportsDataflash);

            if (supportsDataflash) {
                // UI hooks
                $('.tab-dataflash a.erase-flash').click(ask_to_erase_flash);
                
                $('.tab-dataflash a.erase-flash-confirm').click(flash_erase);
                $('.tab-dataflash a.erase-flash-cancel').click(flash_erase_cancel);
        
                $('.tab-dataflash a.save-flash').click(flash_save_begin);
                $('.tab-dataflash a.save-flash-cancel').click(flash_save_cancel);
                $('.tab-dataflash a.save-flash-dismiss').click(dismiss_saving_dialog);
                
                update_html();
            } else {
                $(".tab-dataflash .note").html(chrome.i18n.getMessage('dataflashNotSupportedNote'));
            }
        } else {
            $(".tab-dataflash").removeClass("supported");
            $(".tab-dataflash .note").html(chrome.i18n.getMessage('dataflashFirmwareUpgradeRequired'));
        }

        
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
    
    function flash_save_cancel() {
        saveCancelled = true;
    }
    
    function show_saving_dialog() {
        $(".dataflash-saving progress").attr("value", 0);
        saveCancelled = false;
        $(".dataflash-saving").removeClass("done");
        
        $(".dataflash-saving")[0].showModal();
    }
    
    function dismiss_saving_dialog() {
        $(".dataflash-saving")[0].close();
    }
    
    function mark_saving_dialog_done() {
        $(".dataflash-saving").addClass("done");
    }
    
    function flash_save_begin() {
        var
            maxBytes = DATAFLASH.usedSize;
        
        if (GUI.connected_to) {
            prepare_file(function(fileWriter) {
                var
                    nextAddress = 0;
                
                show_saving_dialog();
                
                function onChunkRead(chunkAddress, chunkDataView) {
                    if (chunkDataView != null) {
                        // Did we receive any data?
                        if (chunkDataView.byteLength > 0) {
                            nextAddress += chunkDataView.byteLength;
                            
                            $(".dataflash-saving progress").attr("value", nextAddress / maxBytes * 100);
    
                            var 
                                blob = new Blob([chunkDataView]);
                            
                            fileWriter.onwriteend = function(e) {
                                if (saveCancelled || nextAddress >= maxBytes) {
                                    if (saveCancelled) {
                                        dismiss_saving_dialog();
                                    } else {
                                        mark_saving_dialog_done();
                                    }
                                } else {
                                    MSP.dataflashRead(nextAddress, onChunkRead);
                                }
                            };
                            
                            fileWriter.write(blob);
                        } else {
                            // A zero-byte block indicates end-of-file, so we're done
                            mark_saving_dialog_done();
                        }
                    } else {
                        // There was an error with the received block (address didn't match the one we asked for), retry
                        MSP.dataflashRead(nextAddress, onChunkRead);
                    }
                }
                
                // Fetch the initial block
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
            var error = chrome.runtime.lastError;
            
            if (error) {
                console.error(error.message);
                
                if (error.message != "User cancelled") {
                    GUI.log(chrome.i18n.getMessage('dataflashFileWriteFailed'));
                }
                return;
            }
            
            // echo/console log path specified
            chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                console.log('Dataflash dump file path: ' + path);
            });

            fileEntry.createWriter(function (fileWriter) {
                fileWriter.onerror = function (e) {
                    console.error(e);

                    // stop logging if the procedure was/is still running
                };

                onComplete(fileWriter);
            }, function (e) {
                // File is not readable or does not exist!
                console.error(e);
                GUI.log(chrome.i18n.getMessage('dataflashFileWriteFailed'));
            });
        });
    }
    
    function ask_to_erase_flash() {
        eraseCancelled = false;
        $(".dataflash-confirm-erase").removeClass('erasing');

        $(".dataflash-confirm-erase")[0].showModal(); 
    }

    function poll_for_erase_completion() {
        MSP.send_message(MSP_codes.MSP_DATAFLASH_SUMMARY, false, false, function() {
            update_html();
            if (!eraseCancelled) {
                if (DATAFLASH.ready) {
                    $(".dataflash-confirm-erase")[0].close();
                } else {
                    setTimeout(poll_for_erase_completion, 500);
                }
            }
        });
    }
    
    function flash_erase() {
        $(".dataflash-confirm-erase").addClass('erasing');
        
        MSP.send_message(MSP_codes.MSP_DATAFLASH_ERASE, false, false, poll_for_erase_completion);
    }
    
    function flash_erase_cancel() {
        eraseCancelled = true;
        $(".dataflash-confirm-erase")[0].close();
    }
};

TABS.dataflash.cleanup = function (callback) {
    if (callback) callback();
};