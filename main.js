// Get access to the background window object
// This object is used to pass current connectionId to the backround page
// so the onClosed event can close the port for us if it was left opened, without this
// users can experience weird behavior if they would like to access the serial bus afterwards.
chrome.runtime.getBackgroundPage(function(result) {
    backgroundPage = result;
    backgroundPage.app_window = window;
});

var timers = new Array();

function disable_timers() {
    for (var i = 0; i < timers.length; i++) {
        clearInterval(timers[i]);
    }
    
    // kill all the refferences
    timers = [];
    
    return true;
}

// Google Analytics stuff begin
var service = analytics.getService('ice_cream_app');
var ga_tracker = service.getTracker('UA-32728876-6');

ga_tracker.sendAppView('Application Started');
// Google Analytics stuff end    

$(document).ready(function() { 
    var tabs = $('#tabs > ul');
    $('a', tabs).click(function() {
        if ($(this).parent().hasClass('active') == false) { // only initialize when the tab isn't already active
            if (connectionId < 1 || configuration_received == false) { // if there is no active connection, return
                notify('You need to connect before you can view any of the tabs', 'red');
                return;
            }
            
            var self = this;
            
            // Disable any active "data pulling" timer
            disable_timers();
            
            GUI.tab_switch_cleanup(function() {
                // Disable previous active button
                $('li', tabs).removeClass('active');
                
                // Highlight selected button
                $(self).parent().addClass('active');
                
                if ($(self).parent().hasClass('tab_initial_setup')) {
                    $('#content').load("./tabs/initial_setup.html", tab_initialize_initial_setup);
                } else if ($(self).parent().hasClass('tab_pid_tuning')) {
                    $('#content').load("./tabs/pid_tuning.html", tab_initialize_pid_tuning);
                } else if ($(self).parent().hasClass('tab_receiver')) {
                    $('#content').load("./tabs/receiver.html", tab_initialize_receiver);
                } else if ($(self).parent().hasClass('tab_auxiliary_configuration')) {
                    $('#content').load("./tabs/auxiliary_configuration.html", tab_initialize_auxiliary_configuration);
                } else if ($(self).parent().hasClass('tab_servos')) {
                    $('#content').load("./tabs/servos.html", tab_initialize_servos);
                } else if ($(self).parent().hasClass('tab_gps')) {
                    $('#content').load("./tabs/gps.html", tab_initialize_gps);
                } else if ($(self).parent().hasClass('tab_motor_outputs')) {
                    $('#content').load("./tabs/motor_outputs.html", tab_initialize_motor_outputs);
                } else if ($(self).parent().hasClass('tab_sensors')) {
                    $('#content').load("./tabs/sensors.html", tab_initialize_sensors);
                } else if ($(self).parent().hasClass('tab_cli')) {
                    $('#content').load("./tabs/cli.html", tab_initialize_cli);
                }
            });
        }
    });
    
    tab_initialize_default();
});

function notify(message, color) {
    $('span.notify').html('<span style="color: ' + color + '">' + message + '</span>');+
    $('span.notify span').fadeOut(5000, function() {
        $(this).remove();
    });
    
}

function microtime() {
    var now = new Date().getTime() / 1000;

    return now;
}

// accepting single level array with "value" as key
function array_difference(firstArray, secondArray) {
    var cloneArray = [];
    
    // create hardcopy
    for (var i = 0; i < firstArray.length; i++) {
        cloneArray.push(firstArray[i]);
    }
    
    for (var i = 0; i < secondArray.length; i++) {
        if (cloneArray.indexOf(secondArray[i]) != -1) {
            cloneArray.splice(cloneArray.indexOf(secondArray[i]), 1);
        }
    }
    
    return cloneArray;
}