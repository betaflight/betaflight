// Get access to the background window object
// This object is used to pass current connectionId to the backround page
// so the onClosed event can close the port for us if it was left opened, without this
// users can experience weird behavior if they would like to access the serial bus afterwards.
chrome.runtime.getBackgroundPage(function(result) {
    backgroundPage = result;
    backgroundPage.app_window = window;
});

// Google Analytics stuff begin
var service = analytics.getService('ice_cream_app');
var ga_tracker = service.getTracker('UA-32728876-6');

ga_tracker.sendAppView('Application Started');
// Google Analytics stuff end    

$(document).ready(function() {
    // set bounds (default 960x600)
    if (screen.height > 600) {
        chrome.app.window.current().setBounds({width: 960, height: 600});
    } else {
        $('div#content').height(280);
        chrome.app.window.current().setBounds({width: 960, height: 400});
    }
    
    var tabs = $('#tabs > ul');
    $('a', tabs).click(function() {
        if ($(this).parent().hasClass('active') == false) { // only initialize when the tab isn't already active
            if (connectionId < 1 || configuration_received == false) { // if there is no active connection, return
                notify('You need to connect before you can view any of the tabs', 'red');
                return;
            }
            
            var self = this;
            
            GUI.tab_switch_cleanup(function() {
                // disable previously active tab highlight
                $('li', tabs).removeClass('active');
                
                // get tab class name (there should be only one class listed)
                var tab = $(self).parent().prop('class');
                
                // Highlight selected tab
                $(self).parent().addClass('active');
                
                switch (tab) {
                    case 'tab_initial_setup':
                        $('#content').load("./tabs/initial_setup.html", tab_initialize_initial_setup);
                        break;
                    case 'tab_pid_tuning':
                        $('#content').load("./tabs/pid_tuning.html", tab_initialize_pid_tuning);
                        break;
                    case 'tab_receiver':
                        $('#content').load("./tabs/receiver.html", tab_initialize_receiver);
                        break;
                    case 'tab_auxiliary_configuration':
                        $('#content').load("./tabs/auxiliary_configuration.html", tab_initialize_auxiliary_configuration);
                        break;
                    case 'tab_servos':
                        $('#content').load("./tabs/servos.html", tab_initialize_servos);
                        break;
                    case 'tab_gps':
                        $('#content').load("./tabs/gps.html", tab_initialize_gps);
                        break;
                    case 'tab_motor_outputs':
                        $('#content').load("./tabs/motor_outputs.html", tab_initialize_motor_outputs);
                        break;
                    case 'tab_sensors':
                        $('#content').load("./tabs/sensors.html", tab_initialize_sensors);
                        break;    
                    case 'tab_cli':
                        $('#content').load("./tabs/cli.html", tab_initialize_cli);
                        break;                         
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