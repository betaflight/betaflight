chrome.app.runtime.onLaunched.addListener(function() {
    chrome.app.window.create('main.html', {
        frame: 'chrome',
        id: 'main-window',
        minWidth: 960,
        maxWidth: 960,
        minHeight: 600,
        maxHeight: 600
    }, function(window_child) {
        window_child.onClosed.addListener(function() {
            // connectionId is passed from the script side through the chrome.runtime.getBackgroundPage refference
            // allowing us to automatically close the port when application shut down
            if (connectionId != -1) {
                chrome.serial.close(connectionId, function(){
                    console.log('CLEANUP: Connection to serial port was opened after application closed, closing the connection.');
                });
            }
        });
    });
});