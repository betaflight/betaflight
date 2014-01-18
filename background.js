function start_app() {
    chrome.app.window.create('main.html', {
        id: 'main-window',
        frame: 'none',
        resizable: false
    }, function(window_child) {
        window_child.onClosed.addListener(function() {
            // connectionId is passed from the script side through the chrome.runtime.getBackgroundPage refference
            // allowing us to automatically close the port when application shut down
            if (app_window.serial.connectionId > 0) {
                chrome.serial.disconnect(app_window.serial.connectionId, function(result) {
                    console.log('SERIAL: Connection closed - ' + result);
                });
            }
        });
    });
}

chrome.app.runtime.onLaunched.addListener(function() {
    start_app();
});

chrome.runtime.onInstalled.addListener(function(details) {
    if (details.reason == 'update') {        
        var manifest = chrome.runtime.getManifest();        
        var options = {
            priority: 0,
            type: 'basic',
            title: 'Baseflight Configurator Update',
            message: 'Application just updated to version: ' + manifest.version,
            iconUrl: '/images/icon_128.png',
            buttons: [{'title': 'Click this button to start the application'}]
        };
        
        chrome.notifications.create('baseflight_update', options, function(notificationId) {
            // empty
        });
    }
});

chrome.notifications.onButtonClicked.addListener(function(notificationId, buttonIndex) {
    if (notificationId == 'baseflight_update') {
        start_app();
    }
});