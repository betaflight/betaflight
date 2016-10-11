'use strict';

var PortUsage = {
    previous_received:  0,
    previous_sent:      0,

    initialize: function() {
        var self = this;

        self.main_timer_reference = setInterval(function() {
            self.update();
        }, 1000);
    },
    update: function() {
        if (serial.bitrate) {
            var port_usage_down = parseInt(((serial.bytesReceived - this.previous_received) * 10 / serial.bitrate) * 100);
            var port_usage_up = parseInt(((serial.bytesSent - this.previous_sent) * 10 / serial.bitrate) * 100);

            this.previous_received = serial.bytesReceived;
            this.previous_sent = serial.bytesSent;

            // update UI
            $('span.port_usage_down').text(chrome.i18n.getMessage('statusbar_usage_download', [port_usage_down]));
            $('span.port_usage_up').text(chrome.i18n.getMessage('statusbar_usage_upload', [port_usage_up]));
        } else {
            $('span.port_usage_down').text(chrome.i18n.getMessage('statusbar_usage_download', [0]));
            $('span.port_usage_up').text(chrome.i18n.getMessage('statusbar_usage_upload', [0]));
        }
    },
    reset: function() {
        this.previous_received = 0;
        this.previous_sent = 0;
    }
};