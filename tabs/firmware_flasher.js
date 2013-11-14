function tab_initialize_firmware_flasher() {
    ga_tracker.sendAppView('Firmware Flasher');
    GUI.active_tab = 'firmware_flasher';
    
    var intel_hex = false; // standard intel hex in string format
    var raw_hex = false; // parsed raw hex in array format
    
    $('#content').load("./tabs/firmware_flasher.html", function() {
        // UI Hooks
        $('a.load_file').click(function() {
            chrome.fileSystem.chooseEntry({type: 'openFile', accepts: [{extensions: ['hex']}]}, function(fileEntry) {
                if (!fileEntry) {
                    // no "valid" file selected/created, aborting
                    console.log('No valid file selected, aborting');
                    return;
                }
                
                chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                    console.log('Loading file from: ' + path);
                    $('span.path').html(path);
                    
                    fileEntry.file(function(file) {
                        var reader = new FileReader();

                        reader.onerror = function (e) {
                            console.error(e);
                        };
                        
                        reader.onloadend = function(e) {
                            console.log('File loaded');
                            STM32.GUI_status('<span style="color: green">Firmware loaded, ready for flashing</span>');
                            
                            intel_hex = e.target.result;
                            raw_hex = read_hex_file(intel_hex);
                            
                            $('span.size').html((raw_hex.length / 1000) + ' kB');
                        };

                        reader.readAsText(file);
                    });
                });
            });
        });
        
        $('a.load_remote_file').click(function() {
            /*  for future use
                $.get('https://api.github.com/repos/multiwii/baseflight/tags', function(data) {
                    console.log(data)
                });
            */
            
            $.get('https://raw.github.com/multiwii/baseflight/master/obj/baseflight.hex', function(data) {
                intel_hex = data;
                raw_hex = read_hex_file(intel_hex);
                
                $('span.path').html('Using remote Firmware');
                $('span.size').html((raw_hex.length / 1000) + ' kB');
                
                STM32.GUI_status('<span style="color: green">Remote Firmware loaded, ready for flashing</span>');
            }).fail(function() {
                STM32.GUI_status('<span style="color: red">Failed to load remote firmware</span>');
            });
        });
        
        $('a.flash_firmware').click(function() {
            if (!GUI.connect_lock) { // button disabled while flashing is in progress
                if (raw_hex != false) {
                    STM32.hex_to_flash = raw_hex.slice(0);
                    
                    STM32.connect();
                } else {
                    STM32.GUI_status('<span style="color: red">Firmware not loaded</span>');
                }
            }
        });
    });
}