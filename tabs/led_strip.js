'use strict';

TABS.led_strip = {
    totalLights: 0,
    currentWire: 0,
    wireMode: false,
    flightModes: ['w', 'f', 'i', 'a', 't'],
    ledOrientations: ['n', 'e', 's', 'w', 'u', 'd'],
};

TABS.led_strip.initialize = function (callback, scrollPosition) {
    var self = this;

    if (GUI.active_tab != 'led_strip') {
        GUI.active_tab = 'led_strip';
        googleAnalytics.sendAppView('LED Strip');
    }

    function load_led_config() {
        MSP.send_message(MSP_codes.MSP_LED_STRIP_CONFIG, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/led_strip.html", process_html);
    }

    load_led_config();
    

    function process_html() {
        
        localize();

        // Build Grid
        var theHTML = [];
        var theHTMLlength = 0;
        for (i=0; i<256; i++) {
            theHTML[theHTMLlength++] = ('<div class="gPoint"><div class="indicators"><span class="north"></span><span class="south"></span><span class="west"></span><span class="east"></span><span class="up">U</span><span class="down">D</span></div><span class="wire"></span></div>');
        }
        $('.mainGrid').html(theHTML.join(''));

        $('.tempOutput').click(function() {
            $(this).select();
        });

        // Clear Button
        $('.funcClear').click(function() {
            $('.gPoint').each(function() {
                if ($(this).is('.ui-selected')) {
                    $(this).removeClass(function(index, theClass) { 
                       theClass = theClass.replace(/(^|\s)+gPoint\s+/, '');                    
                       return theClass;
                    });
                    $(this).addClass('ui-selected');
                    updateBulkCmd();
                }
            });

            $('.controls button').removeClass('btnOn');
        });


        // Directional Buttons
        $('.orientation').on('click', 'button', function() {
            var that = this;
            if ($('.ui-selected').length > 0) {
                TABS.led_strip.ledOrientations.forEach(function(letter) {
                    if ($(that).is('.dir' + letter.toUpperCase())) {
                        $(that).toggleClass('btnOn');
                        $('.ui-selected').toggleClass('dir-' + letter);
                    }
                });

                updateBulkCmd();
            }
        });


        // Mode Buttons
        $('.modes').on('click', 'button', function() {
            var that = this;
            if ($('.ui-selected').length > 0) {
                TABS.led_strip.flightModes.forEach(function(letter) {
                    if ($(that).is('.mode' + letter.toUpperCase())) {
                        $(that).toggleClass('btnOn');
                        $('.ui-selected').toggleClass('mode-' + letter);
                    }
                });

                updateBulkCmd();
            }
        });

        $('.funcWire').click(function() {
            (TABS.led_strip.wireMode) ? TABS.led_strip.wireMode=false : TABS.led_strip.wireMode=true;
            $(this).toggleClass('btnOn');
            $('.mainGrid').toggleClass('gridWire');
        });

        $('.funcWireClearSelect').click(function() {
            $('.ui-selected').each(function() {
                var thisWire = $(this).find('.wire');
                if (thisWire.html() != '') {
                    thisWire.html('');
                    TABS.led_strip.currentWire--;
                }
                updateBulkCmd();             
            });
        });

        $('.funcWireClear').click(function() {
            TABS.led_strip.currentWire = 0;
            $('.gPoint .wire').html('');
            updateBulkCmd();
        });


        $('.mainGrid').selectable({
            filter: ' > div',
            stop: function() {
                $('.ui-selected').each(function() {             
                    if (TABS.led_strip.wireMode) {
                        if ($(this).find('.wire').html() == '' && TABS.led_strip.currentWire < LED_STRIP.length) {
                            $(this).find('.wire').html(TABS.led_strip.currentWire);
                            TABS.led_strip.currentWire++;
                        }
                    }

                    var that = this;

                    TABS.led_strip.ledOrientations.forEach(function(letter) {
                        if ($(that).is('.dir-' + letter)) {
                            $('.dir' + letter.toUpperCase()).addClass('btnOn');
                        } else {
                            $('.dir' + letter.toUpperCase()).removeClass('btnOn');
                        }
                    });

                    TABS.led_strip.flightModes.forEach(function(letter) {
                        if ($(that).is('.mode-' + letter)) {
                            $('.mode' + letter.toUpperCase()).addClass('btnOn');
                        } else {
                            $('.mode' + letter.toUpperCase()).removeClass('btnOn');
                        }
                    });

                    updateBulkCmd();
                });
            }
        });

        $('.mainGrid').disableSelection();

        $('.gPoint').each(function(){
            var gridNumber = ($(this).index() + 1);
            var row = Math.ceil(gridNumber / 16) - 1;
            var col = gridNumber/16 % 1 * 16 - 1;
            if (col < 0) {
                col = 15;
            }
            
            var ledResult = findLed(col, row);
            if (!ledResult) {
                return;
            }
            
            var ledIndex = ledResult.index;
            var led = ledResult.led;
            
            if (led.functions.length == 0 && led.directions.length == 0) {
                return;
            }
            
            $(this).find('.wire').html(ledIndex);

            for (var modeIndex = 0; modeIndex < led.functions.length; modeIndex++) {
                $(this).addClass('mode-' + led.functions[modeIndex]);
            }
            
            for (var directionIndex = 0; directionIndex < led.directions.length; directionIndex++) {
                $(this).addClass('dir-' + led.directions[directionIndex]);
            }

        });
        updateBulkCmd(); 
        
        if (callback) callback();
    }
    
    function findLed(x, y) {
        for (var ledIndex = 0; ledIndex < LED_STRIP.length; ledIndex++) {
            var led = LED_STRIP[ledIndex];
            if (led.x == x && led.y == y) {
                return { index: ledIndex, led: led };
            }
        }
        return undefined;
    }
        
    function updateBulkCmd() {
        $('.tempOutput').empty();
        $('.tempOutput').html('# Copy and paste commands below into the CLI' + "\n\n");
        var counter = 0;

        var lines = [];
        
        $('.gPoint').each(function(){
            if ($(this).is('[class*="mode"]')) {
                var gridNumber = ($(this).index() + 1);
                var row = Math.ceil(gridNumber / 16) - 1;
                var col = gridNumber/16 % 1 * 16 - 1;
                if (col < 0) {col = 15;}

                var wireNumber = $(this).find('.wire').html();
                var ledModes = '';
                var directions = '';
                var that = this;

                TABS.led_strip.flightModes.forEach(function(letter){
                    if ($(that).is('.mode-' + letter)) {
                        ledModes += letter.toUpperCase();
                    }
                });

                TABS.led_strip.ledOrientations.forEach(function(letter){
                    if ($(that).is('.dir-' + letter)) {
                        directions += letter.toUpperCase();
                    }
                });

                if (wireNumber != '') {
                    var line = 'led ' + wireNumber + ' ' + col + ',' + row + ':' + directions + ':' + ledModes;
                    lines[wireNumber] = line;
                }
                counter++;
            }
        });

        $('.tempOutput').append(lines.join("\n"));

        TABS.led_strip.totalLights = counter;

        var remaining = LED_STRIP.length - TABS.led_strip.totalLights;
        if (remaining <= 0) {
            remaining = 0;
            $('.wires-remaining').addClass('error');
        } else {
            $('.wires-remaining').removeClass('error');
        }
        $('.wires-remaining div').html(remaining);
    }

};

TABS.led_strip.cleanup = function (callback) {
    if (callback) callback();
};
