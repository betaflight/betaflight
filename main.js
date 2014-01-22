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
        chrome.app.window.current().setBounds({width: 962, height: 627});
    } else {
        $('div#content').height(280);
        chrome.app.window.current().setBounds({width: 960, height: 427});
    }
    
    // bind controls  
    $('#frame .minimize').click(function() {
        chrome.app.window.current().minimize();
    }); 

    $('#frame .maximize').click(function() {
    });
    
    $('#frame .close').click(function() {
        chrome.app.window.current().close();
    });    
    
    // Tabs
    var tabs = $('#tabs > ul');
    $('a', tabs).click(function() {
        if ($(this).parent().hasClass('active') == false) { // only initialize when the tab isn't already active
            if (configuration_received == false) { // if there is no active connection, return
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
    
    // listen to all input change events and adjust the value within limits if necessary
    $("#content").on("focus", 'input[type="number"]', function() {
        var element = $(this);
        var val = element.val();
        
        if (!isNaN(val)) {
            element.data('previousValue', parseFloat(val));
        }
    });
    
    $("#content").on("change", 'input[type="number"]', function() {
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
    });
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

function add_custom_spinners() {
    var spinner_element = '<div class="spinner"><div class="up"></div><div class="down"></div></div>';
    
    $('input[type="number"]').each(function() {
        var input = $(this);
        var isInt =(input.prop('step') == '') ? true : false;
        
        // make space for spinner
        input.width(input.width() - 16);
        
        // add spinner
        input.after(spinner_element);
        
        // get spinner refference
        var spinner = input.next();
        
        // bind UI hooks to spinner
        $('.up', spinner).click(function() {
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
        });
        
        $('.down', spinner).click(function() {
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
        });
    });
}