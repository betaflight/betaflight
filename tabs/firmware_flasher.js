function tab_initialize_firmware_flasher() {
    ga_tracker.sendAppView('Firmware Flasher');
    GUI.active_tab = 'firmware_flasher';
    
    var intel_hex = false;
    
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
                    
                    fileEntry.file(function(file) {
                        var reader = new FileReader();

                        reader.onerror = function (e) {
                            console.error(e);
                        };
                        
                        reader.onloadend = function(e) {
                            console.log('File loaded');
                            
                            intel_hex = e.target.result;
                        };

                        reader.readAsText(file);
                    });
                });
            });
        });
    });
}