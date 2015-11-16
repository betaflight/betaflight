/*
    If an id is also specified and a window with a matching id has been shown before, the remembered bounds of the window will be used instead.
*/
'use strict';

function startApplication() {
    var applicationStartTime = new Date().getTime();

    chrome.app.window.create('main.html', {
        id: 'main-window',
        frame: 'chrome',
        innerBounds: {
            minWidth: 1024,
            minHeight: 550
        }
    }, function (createdWindow) {
        createdWindow.contentWindow.addEventListener('load', function () {
            createdWindow.contentWindow.catch_startup_time(applicationStartTime);
        });

        createdWindow.onClosed.addListener(function () {
            // automatically close the port when application closes
            // save connectionId in separate variable before createdWindow.contentWindow is destroyed
            var connectionId = createdWindow.contentWindow.serial.connectionId,
                valid_connection = createdWindow.contentWindow.CONFIGURATOR.connectionValid,
                mincommand = createdWindow.contentWindow.MISC.mincommand;

            if (connectionId && valid_connection) {
                // code below is handmade MSP message (without pretty JS wrapper), it behaves exactly like MSP.send_message
                // sending exit command just in case the cli tab was open.
                // reset motors to default (mincommand)

                var bufferOut = new ArrayBuffer(5),
                bufView = new Uint8Array(bufferOut);

                bufView[0] = 0x65; // e
                bufView[1] = 0x78; // x
                bufView[2] = 0x69; // i
                bufView[3] = 0x74; // t
                bufView[4] = 0x0D; // enter

                chrome.serial.send(connectionId, bufferOut, function () { console.log('Send exit') }); 

                setTimeout(function() {
                    bufferOut = new ArrayBuffer(22);
                    bufView = new Uint8Array(bufferOut);
                    var checksum = 0;

                    bufView[0] = 36; // $
                    bufView[1] = 77; // M
                    bufView[2] = 60; // <
                    bufView[3] = 16; // data length
                    bufView[4] = 214; // MSP_SET_MOTOR

                    checksum = bufView[3] ^ bufView[4];

                    for (var i = 0; i < 16; i += 2) {
                        bufView[i + 5] = mincommand & 0x00FF;
                        bufView[i + 6] = mincommand >> 8;

                        checksum ^= bufView[i + 5];
                        checksum ^= bufView[i + 6];
                    }

                    bufView[5 + 16] = checksum;

                    chrome.serial.send(connectionId, bufferOut, function (sendInfo) {
                        chrome.serial.disconnect(connectionId, function (result) {
                            console.log('SERIAL: Connection closed - ' + result);
                        });
                    });
                }, 100);
            } else if (connectionId) {
                chrome.serial.disconnect(connectionId, function (result) {
                    console.log('SERIAL: Connection closed - ' + result);
                });
            }
        });
    });
}

chrome.app.runtime.onLaunched.addListener(startApplication);

chrome.runtime.onInstalled.addListener(function (details) {
    if (details.reason == 'update') {
        var previousVersionArr = details.previousVersion.split('.'),
            currentVersionArr = chrome.runtime.getManifest().version.split('.');

        // only fire up notification sequence when one of the major version numbers changed
        if (currentVersionArr[0] > previousVersionArr[0] || currentVersionArr[1] > previousVersionArr[1]) {
            chrome.storage.local.get('update_notify', function (result) {
                if (result.update_notify === 'undefined' || result.update_notify) {
                    var manifest = chrome.runtime.getManifest();
                    var options = {
                        priority: 0,
                        type: 'basic',
                        title: manifest.name,
                        message: chrome.i18n.getMessage('notifications_app_just_updated_to_version', [manifest.version]),
                        iconUrl: '/images/icon_128.png',
                        buttons: [{'title': chrome.i18n.getMessage('notifications_click_here_to_start_app')}]
                    };

                    chrome.notifications.create('baseflight_update', options, function (notificationId) {
                        // empty
                    });
                }
            });
        }
    }
});

chrome.notifications.onButtonClicked.addListener(function (notificationId, buttonIndex) {
    if (notificationId == 'baseflight_update') {
        startApplication();
    }
});