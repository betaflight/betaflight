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
    // bind controls  
    $('#frame .minimize').click(function() {
        chrome.app.window.current().minimize();
    }); 

    $('#frame .maximize').click(function() {
    });
    
    $('#frame .close').click(function() {
        chrome.app.window.current().close();
    });
    
    // alternative - window.navigator.appVersion.match(/Chrome\/([0-9.]*)/)[1];
    GUI.log('Running - OS: <strong>' + GUI.operating_system + '</strong>, ' + 
        'Chrome: <strong>' + window.navigator.appVersion.replace(/.*Chrome\/([0-9.]*).*/,"$1") + '</strong>, ' +
        'Configurator: <strong>' + chrome.runtime.getManifest().version + '</strong>');

    // notification messages for various operating systems
    switch (GUI.operating_system) {
        case 'Windows':
            break;
        case 'MacOS':
            var main_chromium_version = window.navigator.appVersion.replace(/.*Chrome\/([0-9.]*).*/,"$1").split('.')[0];
            
            if (main_chromium_version >= 33) {
                GUI.log('Configurator detected that you are running Chrome / Chromium with broken <strong>Serial API</strong> which \
                <strong style="color: red">breaks</strong> the communication completely on this platform');
                GUI.log('You can find more about this issue on this address \
                <strong>"<a href="https://code.google.com/p/chromium/issues/detail?id=337482" title="Chromium Bug Tracker" target="_blank">new serial API fails to set higher baudrate then 38400</a>"</strong>');
            }
            break;
        case 'ChromeOS':
            break;
        case 'Linux':
            break;
        case 'UNIX':
            break;
    }
    
    // Tabs
    var tabs = $('#tabs > ul');
    $('a', tabs).click(function() {
        if ($(this).parent().hasClass('active') == false) { // only initialize when the tab isn't already active
            if (configuration_received == false) { // if there is no active connection, return
                GUI.log('You need to connect before you can view any of the tabs', 'red');
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
                        tab_initialize_initial_setup();
                        break;
                    case 'tab_pid_tuning':
                        tab_initialize_pid_tuning();
                        break;
                    case 'tab_receiver':
                        tab_initialize_receiver();
                        break;
                    case 'tab_auxiliary_configuration':
                        tab_initialize_auxiliary_configuration();
                        break;
                    case 'tab_servos':
                        tab_initialize_servos();
                        break;
                    case 'tab_gps':
                        tab_initialize_gps();
                        break;
                    case 'tab_motor_outputs':
                        tab_initialize_motor_outputs();
                        break;
                    case 'tab_sensors':
                        tab_initialize_sensors();
                        break;    
                    case 'tab_cli':
                        tab_initialize_cli();
                        break;                         
                }
            });
        }
    });
    
    tab_initialize_default();
    
    // listen to all input change events and adjust the value within limits if necessary
    $("#content").on('focus', 'input[type="number"]', function() {
        var element = $(this);
        var val = element.val();
        
        if (!isNaN(val)) {
            element.data('previousValue', parseFloat(val));
        }
    });
    
    $("#content").on('keydown', 'input[type="number"]', function(e) {
        // whitelist all that we need for numeric control
        if ((e.keyCode >= 96 && e.keyCode <= 105) || (e.keyCode >= 48 && e.keyCode <= 57)) { // allow numpad and standard number keypad
        } else if(e.keyCode == 109 || e.keyCode == 189) { // minus on numpad and in standard keyboard
        } else if (e.keyCode == 8 || e.keyCode == 46) { // backspace and delete
        } else if (e.keyCode == 190 || e.keyCode == 110) { // allow and decimal point
        } else if ((e.keyCode >= 37 && e.keyCode <= 40) || e.keyCode == 13) { // allow arrows, enter
        } else {
            // block everything else
            e.preventDefault();
        }
    });
    
    $("#content").on('change', 'input[type="number"]', function() {
        var element = $(this);
        var min = parseFloat(element.prop('min'));
        var max = parseFloat(element.prop('max'));
        var step = parseFloat(element.prop('step'));
        var val = parseFloat(element.val());
        
        // only adjust minimal end if bound is set
        if (element.prop('min')) {
            if (val < min) element.val(min);
        }
        
        // only adjust maximal end if bound is set
        if (element.prop('max')) {
            if (val > max) element.val(max);
        }
        
        // if entered value is illegal use previous value instead
        if (isNaN(val)) {
            element.val(element.data('previousValue'));
        }
        
        // if step is not set or step is int and value is float use previous value instead
        if (isNaN(step) || step % 1 === 0) {
            if (val % 1 !== 0) {
                element.val(element.data('previousValue'));
            }
        }
        
        // if step is set and is float and value is int, convert to float, keep decimal places in float according to step *experimental*
        if (!isNaN(step) && step % 1 !== 0) {
            var decimal_places = String(step).split('.')[1].length;
            
            if (val % 1 === 0) {
                element.val(val.toFixed(decimal_places));
            } else if (String(val).split('.')[1].length != decimal_places) {
                element.val(val.toFixed(decimal_places));
            }
        }
    });
});

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

/*
function add_custom_spinners() {
    var spinner_element = '<div class="spinner"><div class="up"></div><div class="down"></div></div>';
    
    $('input[type="number"]').each(function() {
        var input = $(this);
        
        // only add new spinner if one doesn't already exist
        if (!input.next().hasClass('spinner')) {
            var isInt = true;
            if (input.prop('step') == '') {
                isInt = true;
            } else {
                if (input.prop('step').indexOf('.') == -1) {
                    isInt = true;
                } else {
                    isInt = false;
                }
            }
            
            // make space for spinner
            input.width(input.width() - 16);
            
            // add spinner
            input.after(spinner_element);
            
            // get spinner refference
            var spinner = input.next();
            
            // bind UI hooks to spinner
            $('.up', spinner).click(function() {
                up();
            });
            
            $('.up', spinner).mousedown(function() {            
                GUI.timeout_add('spinner', function() {
                    GUI.interval_add('spinner', function() {
                        up();
                    }, 100, true);
                }, 250);
            });
            
            $('.up', spinner).mouseup(function() {
                GUI.timeout_remove('spinner');
                GUI.interval_remove('spinner');
            });
            
            $('.up', spinner).mouseleave(function() {            
                GUI.timeout_remove('spinner');
                GUI.interval_remove('spinner');
            });
            
            
            $('.down', spinner).click(function() {
                down();
            });
            
            $('.down', spinner).mousedown(function() {            
                GUI.timeout_add('spinner', function() {
                    GUI.interval_add('spinner', function() {
                        down();
                    }, 100, true);
                }, 250);
            });
            
            $('.down', spinner).mouseup(function() {
                GUI.timeout_remove('spinner');
                GUI.interval_remove('spinner');
            });
            
            $('.down', spinner).mouseleave(function() {
                GUI.timeout_remove('spinner');
                GUI.interval_remove('spinner');
            });
            
            var up = function() {
                if (isInt) {
                    var current_value = parseInt(input.val());
                    input.val(current_value + 1);
                } else {
                    var current_value = parseFloat(input.val());
                    var step = parseFloat(input.prop('step'));
                    var step_decimals = input.prop('step').length - 2;
                    
                    input.val((current_value + step).toFixed(step_decimals));
                }
                
                input.change();
            };
            
            var down = function() {
                if (isInt) {
                    var current_value = parseInt(input.val());
                    input.val(current_value - 1);
                } else {
                    var current_value = parseFloat(input.val());
                    var step = parseFloat(input.prop('step'));
                    var step_decimals = input.prop('step').length - 2;
                    
                    input.val((current_value - step).toFixed(step_decimals));
                }
                
                input.change();
            };
        }
    });
}
*/