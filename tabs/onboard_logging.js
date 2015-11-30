'use strict';

var
    sdcardTimer;

TABS.onboard_logging = {
    available: false
};
TABS.onboard_logging.initialize = function (callback) {
    var 
        self = this,
        saveCancelled, eraseCancelled;

    if (GUI.active_tab != 'onboard_logging') {
        GUI.active_tab = 'onboard_logging';
        googleAnalytics.sendAppView('onboard_logging');
    }

    if (CONFIGURATOR.connectionValid) {
        // Blackbox was introduced in 1.5.0, dataflash API was introduced in 1.8.0, BLACKBOX/SDCARD MSP APIs in 1.11.0
        TABS.onboard_logging.available = semver.gte(CONFIG.flightControllerVersion, "1.5.0");
        
        if (!TABS.onboard_logging.available) {
            load_html();
            return;
        }
        
        MSP.send_message(MSP_codes.MSP_BF_CONFIG, false, false, function() {
            if (semver.gte(CONFIG.flightControllerVersion, "1.8.0")) {
                MSP.send_message(MSP_codes.MSP_DATAFLASH_SUMMARY, false, false, function() {
                    if (semver.gte(CONFIG.flightControllerVersion, "1.11.0")) {
                        MSP.send_message(MSP_codes.MSP_SDCARD_SUMMARY, false, false, function() {
                            MSP.send_message(MSP_codes.MSP_BLACKBOX_CONFIG, false, false, load_html);
                        });
                    } else {
                        load_html();
                    }
                });
            } else {
                load_html();
            }
        });
    }
    
    function gcd(a, b) {
        if (b == 0)
            return a;
        
        return gcd(b, a % b);
    }
    
    function save_to_eeprom() {
        MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, reboot);
    }

    function reboot() {
        GUI.log(chrome.i18n.getMessage('configurationEepromSaved'));

        GUI.tab_switch_cleanup(function() {
            MSP.send_message(MSP_codes.MSP_SET_REBOOT, false, false, reinitialize);
        });
    }

    function reinitialize() {
        GUI.log(chrome.i18n.getMessage('deviceRebooting'));

        if (BOARD.find_board_definition(CONFIG.boardIdentifier).vcp) { // VCP-based flight controls may crash old drivers, we catch and reconnect
            $('a.connect').click();
            GUI.timeout_add('start_connection',function start_connection() {
                $('a.connect').click();
            },2000);
        } else {

            GUI.timeout_add('waiting_for_bootup', function waiting_for_bootup() {
                MSP.send_message(MSP_codes.MSP_IDENT, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('deviceReady'));
                    TABS.onboard_logging.initialize(false, $('#content').scrollTop());
                });
            },1500); // 1500 ms seems to be just the right amount of delay to prevent data request timeouts
        }
    }
    
    function load_html() {
        $('#content').load("./tabs/onboard_logging.html", function() {
            // translate to user-selected language
            localize();
           
            var 
                dataflashPresent = DATAFLASH.totalSize > 0,
                blackboxSupport;
                
            /* 
             * Pre-1.11.0 firmware supported DATAFLASH API (on targets with SPI flash) but not the BLACKBOX config API.
             * 
             * The best we can do on those targets is check the BLACKBOX feature bit to identify support for Blackbox instead.
             */
            if (BLACKBOX.supported || DATAFLASH.supported 
                    || semver.gte(CONFIG.flightControllerVersion, "1.5.0") && semver.lte(CONFIG.flightControllerVersion, "1.10.0") && bit_check(BF_CONFIG.features, 19)) {
                blackboxSupport = 'yes';
            } else if (semver.gte(CONFIG.flightControllerVersion, "1.5.0") && semver.lte(CONFIG.flightControllerVersion, "1.10.0")) {
                blackboxSupport = 'maybe';
            } else {
                blackboxSupport = 'no';
            }
            
            $(".tab-onboard_logging")
                .addClass("serial-supported")
                .toggleClass("dataflash-supported", DATAFLASH.supported)
                .toggleClass("dataflash-present", dataflashPresent)
                .toggleClass("sdcard-supported", SDCARD.supported)
                .toggleClass("blackbox-config-supported", BLACKBOX.supported)
                
                .toggleClass("blackbox-supported", blackboxSupport == 'yes')
                .toggleClass("blackbox-maybe-supported", blackboxSupport == 'maybe')
                .toggleClass("blackbox-unsupported", blackboxSupport == 'no');

            if (dataflashPresent) {
                // UI hooks
                $('.tab-onboard_logging a.erase-flash').click(ask_to_erase_flash);
                
                $('.tab-onboard_logging a.erase-flash-confirm').click(flash_erase);
                $('.tab-onboard_logging a.erase-flash-cancel').click(flash_erase_cancel);
        
                $('.tab-onboard_logging a.save-flash').click(flash_save_begin);
                $('.tab-onboard_logging a.save-flash-cancel').click(flash_save_cancel);
                $('.tab-onboard_logging a.save-flash-dismiss').click(dismiss_saving_dialog);
            }
            
            if (BLACKBOX.supported) {
                $(".tab-onboard_logging a.save-settings").click(function() {
                    var rate = $(".blackboxRate select").val().split('/');
                    
                    BLACKBOX.blackboxRateNum = parseInt(rate[0], 10);
                    BLACKBOX.blackboxRateDenom = parseInt(rate[1], 10);
                    BLACKBOX.blackboxDevice = parseInt($(".blackboxDevice select").val(), 10);
                    
                    MSP.sendBlackboxConfiguration(save_to_eeprom);
                });
            }
            
            populateLoggingRates();
            populateDevices();
            
            update_html();
            
            GUI.content_ready(callback);
        });
    }
    
    function populateDevices() {
        var
            deviceSelect = $(".blackboxDevice select").empty();
        
        deviceSelect.append('<option value="0">Serial port</option>');
        if (DATAFLASH.ready) {
            deviceSelect.append('<option value="1">On-board dataflash chip</option>');
        }
        if (SDCARD.supported) {
            deviceSelect.append('<option value="2">On-board SD card slot</option>');
        }
        
        deviceSelect.val(BLACKBOX.blackboxDevice);
    }
    
    function populateLoggingRates() {
        var
            userRateGCD = gcd(BLACKBOX.blackboxRateNum, BLACKBOX.blackboxRateDenom),
            userRate = {num: BLACKBOX.blackboxRateNum / userRateGCD, denom: BLACKBOX.blackboxRateDenom / userRateGCD};
        
        // Offer a reasonable choice of logging rates (if people want weird steps they can use CLI)
        var 
            loggingRates = [
                 {num: 1, denom: 32},
                 {num: 1, denom: 16},
                 {num: 1, denom: 8},
                 {num: 1, denom: 5},
                 {num: 1, denom: 4},
                 {num: 1, denom: 3},
                 {num: 1, denom: 2},
                 {num: 2, denom: 3},
                 {num: 3, denom: 4},
                 {num: 4, denom: 5},
                 {num: 7, denom: 8},
                 {num: 1, denom: 1},
            ],
            loggingRatesSelect = $(".blackboxRate select");
        
        var
            addedCurrentValue = false;
        
        for (var i = 0; i < loggingRates.length; i++) {
            if (!addedCurrentValue && userRate.num / userRate.denom <= loggingRates[i].num / loggingRates[i].denom) {
                if (userRate.num / userRate.denom < loggingRates[i].num / loggingRates[i].denom) {
                    loggingRatesSelect.append('<option value="' + userRate.num + '/' + userRate.denom + '">' 
                            + userRate.num + '/' + userRate.denom + ' (' + Math.round(userRate.num / userRate.denom * 100) + '%)</option>');
                }
                addedCurrentValue = true;
            }
            
            loggingRatesSelect.append('<option value="' + loggingRates[i].num + '/' + loggingRates[i].denom + '">' 
                + loggingRates[i].num + '/' + loggingRates[i].denom + ' (' + Math.round(loggingRates[i].num / loggingRates[i].denom * 100) + '%)</option>');
            
        }
        loggingRatesSelect.val(userRate.num + '/' + userRate.denom);
    }
    
    function formatFilesizeKilobytes(kilobytes) {
        if (kilobytes < 1024) {
            return Math.round(kilobytes) + "kB";
        }
        
        var 
            megabytes = kilobytes / 1024,
            gigabytes;
        
        if (megabytes < 900) {
            return megabytes.toFixed(1) + "MB";
        } else {
            gigabytes = megabytes / 1024;
            
            return gigabytes.toFixed(1) + "GB";
        }
    }
    
    function formatFilesizeBytes(bytes) {
        if (bytes < 1024) {
            return bytes + "B";
        }
        return formatFilesizeKilobytes(bytes / 1024);
    }
    
    function update_bar_width(bar, value, total, label, valuesAreKilobytes) {
        if (value > 0) {
            bar.css({
                width: (value / total * 100) + "%",
                display: 'block'
            });
            
            $("div", bar).text((label ? label + " " : "") + (valuesAreKilobytes ? formatFilesizeKilobytes(value) : formatFilesizeBytes(value)));
        } else {
            bar.css({
                display: 'none'
            });
        }
    }
    
    function update_html() {
        update_bar_width($(".tab-onboard_logging .dataflash-used"), DATAFLASH.usedSize, DATAFLASH.totalSize, "Used space", false);
        update_bar_width($(".tab-onboard_logging .dataflash-free"), DATAFLASH.totalSize - DATAFLASH.usedSize, DATAFLASH.totalSize, "Free space", false);

        update_bar_width($(".tab-onboard_logging .sdcard-other"), SDCARD.totalSizeKB - SDCARD.freeSizeKB, SDCARD.totalSizeKB, "Unavailable space", true);
        update_bar_width($(".tab-onboard_logging .sdcard-free"), SDCARD.freeSizeKB, SDCARD.totalSizeKB, "Free space for logs", true);

        $(".btn a.erase-flash, .btn a.save-flash").toggleClass("disabled", DATAFLASH.usedSize == 0);
        
        $(".tab-onboard_logging")
            .toggleClass("sdcard-error", SDCARD.state == MSP.SDCARD_STATE_FATAL)
            .toggleClass("sdcard-initializing", SDCARD.state == MSP.SDCARD_STATE_CARD_INIT || SDCARD.state == MSP.SDCARD_STATE_FS_INIT)
            .toggleClass("sdcard-ready", SDCARD.state == MSP.SDCARD_STATE_READY);
        
        switch (SDCARD.state) {
            case MSP.SDCARD_STATE_NOT_PRESENT:
                $(".sdcard-status").text("No card inserted");
            break;
            case MSP.SDCARD_STATE_FATAL:
                $(".sdcard-status").html("Fatal error<br>Reboot to retry");
            break;
            case MSP.SDCARD_STATE_READY:
                $(".sdcard-status").text("Card ready");
            break;
            case MSP.SDCARD_STATE_CARD_INIT:
                $(".sdcard-status").text("Card starting...");
            break;
            case MSP.SDCARD_STATE_FS_INIT:
                $(".sdcard-status").text("Filesystem starting...");
            break;
            default:
                $(".sdcard-status").text("Unknown state " + SDCARD.state);
        }
        
        if (SDCARD.supported && !sdcardTimer) {
            // Poll for changes in SD card status
            sdcardTimer = setTimeout(function() {
                sdcardTimer = false;
                if (CONFIGURATOR.connectionValid) {
                    MSP.send_message(MSP_codes.MSP_SDCARD_SUMMARY, false, false, function() {
                        update_html();
                    });
                }
            }, 2000);
        }
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
    
    function flash_update_summary(onDone) {
        MSP.send_message(MSP_codes.MSP_DATAFLASH_SUMMARY, false, false, function() {
            update_html();
            
            if (onDone) {
                onDone();
            }
        });
    }
    
    function flash_save_begin() {
        if (GUI.connected_to) {
            // Begin by refreshing the occupied size in case it changed while the tab was open
            flash_update_summary(function() {
                var
                    maxBytes = DATAFLASH.usedSize;
                
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
        flash_update_summary(function() {
            if (CONFIGURATOR.connectionValid && !eraseCancelled) {
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

TABS.onboard_logging.cleanup = function (callback) {
    if (sdcardTimer) {
        clearTimeout(sdcardTimer);
        sdcardTimer = false;
    }
    
    if (callback) {
        callback();
    }
};