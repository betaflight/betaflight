// Get access to the background window object
// This object is used to pass current connectionId to the backround page
// so the onClosed event can close the port for us if it was left opened, without this
// users can experience weird behavior if they would like to access the serial bus afterwards.
chrome.runtime.getBackgroundPage(function(result) {
    backgroundPage = result;
    backgroundPage.app_window = window;
});

// Google Analytics BEGIN
var ga_config; // google analytics config reference
var ga_tracking; // global result of isTrackingPermitted

var service = analytics.getService('ice_cream_app');
service.getConfig().addCallback(function(config) {
    ga_config = config;
    ga_tracking = config.isTrackingPermitted();
});

var ga_tracker = service.getTracker('UA-32728876-6');

ga_tracker.sendAppView('Application Started');
// Google Analytics END

$(document).ready(function() {
    // translate to user-selected language
    localize();

    // alternative - window.navigator.appVersion.match(/Chrome\/([0-9.]*)/)[1];
    GUI.log('Running - OS: <strong>' + GUI.operating_system + '</strong>, ' +
        'Chrome: <strong>' + window.navigator.appVersion.replace(/.*Chrome\/([0-9.]*).*/,"$1") + '</strong>, ' +
        'Configurator: <strong>' + chrome.runtime.getManifest().version + '</strong>');

    // notification messages for various operating systems
    switch (GUI.operating_system) {
        case 'Windows':
            break;
        case 'MacOS':
            // var main_chromium_version = window.navigator.appVersion.replace(/.*Chrome\/([0-9.]*).*/,"$1").split('.')[0];
            break;
        case 'ChromeOS':
            break;
        case 'Linux':
            break;
        case 'UNIX':
            break;
    }

    // Tabs
    var ui_tabs = $('#tabs > ul');
    $('a', ui_tabs).click(function() {
        if ($(this).parent().hasClass('active') == false && !GUI.tab_switch_in_progress) { // only initialize when the tab isn't already active
            var self = this;
            var index = $(self).parent().index();
            var tab = $(self).parent().prop('class');

            // if there is no active connection, return
            if (!configuration_received && tab != 'tab_logging') {
                GUI.log('You need to <strong>connect</strong> before you can view any of the tabs');
                return;
            }

            GUI.tab_switch_in_progress = true;

            GUI.tab_switch_cleanup(function() {
                // disable previously active tab highlight
                $('li', ui_tabs).removeClass('active');

                // Highlight selected tab
                $(self).parent().addClass('active');

                // detach listeners and remove element data
                $('#content').empty();

                switch (tab) {
                    case 'tab_initial_setup':
                        tabs.initial_setup.initialize();
                        break;
                    case 'tab_pid_tuning':
                        tabs.pid_tuning.initialize();
                        break;
                    case 'tab_receiver':
                        tabs.receiver.initialize();
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
                    case 'tab_logging':
                        tab_initialize_logging();
                        break;
                }

                GUI.tab_switch_in_progress = false;
            });
        }
    });

    tabs.default.initialize();

    // options
    $('a#options').click(function() {
        var el = $(this);

        if (!el.hasClass('active')) {
            el.addClass('active');
            el.after('<div id="options-window"></div>');
            $('div#options-window').load('./tabs/options.html', function() {
                ga_tracker.sendAppView('Options');

                // translate to user-selected language
                localize();

                // if notifications are enabled, or wasn't set, check the notifications checkbox
                chrome.storage.local.get('update_notify', function(result) {
                    if (typeof result.update_notify === 'undefined' || result.update_notify) {
                        $('div.notifications input').prop('checked', true);
                    }
                });

                $('div.notifications input').change(function() {
                    var check = $(this).is(':checked');

                    chrome.storage.local.set({'update_notify': check});
                });

                // if tracking is enabled, check the statistics checkbox
                if (ga_tracking == true) {
                    $('div.statistics input').prop('checked', true);
                }

                $('div.statistics input').change(function() {
                    var check = $(this).is(':checked');

                    ga_tracking = check;

                    ga_config.setTrackingPermitted(check);
                });

                $(this).slideDown();
            });
        } else {
            $('div#options-window').slideUp(function() {
                el.removeClass('active');
                $(this).empty().remove();
            });
        }
    });

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
        } else if (e.keyCode == 109 || e.keyCode == 189) { // minus on numpad and in standard keyboard
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

function millitime() {
    var now = new Date().getTime();

    return now;
}

function bytesToSize(bytes) {
    if (bytes < 1024) return bytes + ' Bytes';
    else if (bytes < 1048576) return(bytes / 1024).toFixed(3) + ' KB';
    else if (bytes < 1073741824) return(bytes / 1048576).toFixed(3) + ' MB';
    else return (bytes / 1073741824).toFixed(3) + ' GB';
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