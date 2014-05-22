function tab_initialize_logging() {
    ga_tracker.sendAppView('Logging');
    GUI.active_tab = 'logging';

    $('#content').load("./tabs/logging.html", process_html);

    function process_html() {
        // translate to user-selected language
        localize();

        // UI hooks
        $('a.log_file').click(prepare_file);

        $('a.logging').click(function() {
            if (fileEntry.isFile) {
                // TODO:
                // grab enabled properties
                // grab refresh rate
                // start data polling timer
                // start buffer flushing sequence & timer (keep fileWriter.readyState in mind)
            }
        });
    }

    // IO related methods
    var fileEntry = null;
    var fileWriter = null;

    function prepare_file() {
        // create or load the file
        chrome.fileSystem.chooseEntry({type: 'saveFile', suggestedName: 'bf_data_log', accepts: [{extensions: ['csv']}]}, function(entry) {
            if (!entry) {
                console.log('No file selected');
                return;
            }

            fileEntry = entry;

            // echo/console log path specified
            chrome.fileSystem.getDisplayPath(fileEntry, function(path) {
                console.log('Log file path: ' + path);
            });

            // change file entry from read only to read/write
            chrome.fileSystem.getWritableEntry(fileEntry, function(fileEntryWritable) {
                // check if file is writable
                chrome.fileSystem.isWritableEntry(fileEntryWritable, function(isWritable) {
                    if (isWritable) {
                        fileEntry = fileEntryWritable;

                        prepare_writer();
                    } else {
                        console.log('File appears to be read only, sorry.');
                    }
                });
            });
        });
    }

    function prepare_writer() {
        fileEntry.createWriter(function(writer) {
            fileWriter = writer;

            fileWriter.onerror = function(e) {
                console.error(e);
            };

            fileWriter.onwriteend = function() {
                // console.log('Data written');
            };
        }, function(e) {
            console.error(e);
        });
    }

    function append_to_file(data) {
        if (fileWriter.position < fileWriter.length) {
            fileWriter.seek(fileWriter.length);
        }

        fileWriter.write(new Blob([data + '\n'], {type: 'text/plain'}));
    }
}