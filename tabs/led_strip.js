'use strict';

TABS.led_strip = {
    wireMode: false,
    functions: ['w', 'f', 'i', 'a', 't'],
    directions: ['n', 'e', 's', 'w', 'u', 'd'],
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
    

    function buildUsedWireNumbers() {
        var usedWireNumbers = [];
        $('.mainGrid .gPoint .wire').each(function () {
            var wireNumber = parseInt($(this).html());
            if (wireNumber >= 0) {
                usedWireNumbers.push(wireNumber);
            }
        });
        usedWireNumbers.sort(function(a,b){return a - b});
        return usedWireNumbers;
    }
    
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
        $('.directions').on('click', 'button', function() {
            var that = this;
            if ($('.ui-selected').length > 0) {
                TABS.led_strip.directions.forEach(function(letter) {
                    if ($(that).is('.dir-' + letter)) {
                        $(that).toggleClass('btnOn');
                        $('.ui-selected').toggleClass('dir-' + letter);
                    }
                });

                updateBulkCmd();
            }
        });


        // Mode Buttons
        $('.functions').on('click', 'button', function() {
            var that = this;
            if ($('.ui-selected').length > 0) {
                TABS.led_strip.functions.forEach(function(letter) {
                    if ($(that).is('.function-' + letter)) {
                        $(that).toggleClass('btnOn');
                        $('.ui-selected').toggleClass('function-' + letter);
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
                }
                updateBulkCmd();
            });
        });

        $('.funcWireClear').click(function() {
            $('.gPoint .wire').html('');
            updateBulkCmd();
        });

        $('.mainGrid').selectable({
            filter: ' > div',
            stop: function() {
                $('.ui-selected').each(function() {

                    
                    var usedWireNumbers = buildUsedWireNumbers();
                    
                    var nextWireNumber = 0;
                    for (var nextWireNumber = 0; nextWireNumber < usedWireNumbers.length; nextWireNumber++) {
                        if (usedWireNumbers[nextWireNumber] != nextWireNumber) {
                            break;
                        }
                    }
                        
                    if (TABS.led_strip.wireMode) {
                        if ($(this).find('.wire').html() == '' && nextWireNumber < LED_STRIP.length) {
                            $(this).find('.wire').html(nextWireNumber);
                        }
                    }

                    var that = this;

                    TABS.led_strip.directions.forEach(function(letter) {
                        if ($(that).is('.dir-' + letter)) {
                            $('.dir-' + letter).addClass('btnOn');
                        } else {
                            $('.dir-' + letter).removeClass('btnOn');
                        }
                    });

                    TABS.led_strip.functions.forEach(function(letter) {
                        if ($(that).is('.function-' + letter)) {
                            $('.function-' + letter).addClass('btnOn');
                        } else {
                            $('.function-' + letter).removeClass('btnOn');
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
                $(this).addClass('function-' + led.functions[modeIndex]);
            }
            
            for (var directionIndex = 0; directionIndex < led.directions.length; directionIndex++) {
                $(this).addClass('dir-' + led.directions[directionIndex]);
            }

        });
        updateBulkCmd(); 
        
        $('a.save').click(function () {

            MSP.sendLedStripConfig(save_to_eeprom);

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function() {
                    GUI.log(chrome.i18n.getMessage('ledStripEepromSaved'));
                });
            }

        });
        
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
        var counter = 0;

        var lines = [];
        var ledStripLength = LED_STRIP.length;
        
        LED_STRIP = [];
        
        $('.gPoint').each(function(){
            if ($(this).is('[class*="function"]')) {
                var gridNumber = ($(this).index() + 1);
                var row = Math.ceil(gridNumber / 16) - 1;
                var col = gridNumber/16 % 1 * 16 - 1;
                if (col < 0) {col = 15;}

                var wireNumber = $(this).find('.wire').html();
                var functions = '';
                var directions = '';
                var that = this;

                TABS.led_strip.functions.forEach(function(letter){
                    if ($(that).is('.function-' + letter)) {
                        functions += letter;
                    }
                });

                TABS.led_strip.directions.forEach(function(letter){
                    if ($(that).is('.dir-' + letter)) {
                        directions += letter;
                    }
                });

                if (wireNumber != '') {
                    var led = {
                        x: col,
                        y: row,
                        directions: directions,
                        functions: functions
                    }
                    
                    LED_STRIP[wireNumber] = led;
                }
                counter++;
            }
        });

        var defaultLed = {
            x: 0,
            y: 0,
            directions: '',
            functions: ''
        };
        
        for (var i = 0; i < ledStripLength; i++) {
            if (LED_STRIP[i]) {
                continue;
            }
            LED_STRIP[i] = defaultLed;
        }
        
        var usedWireNumbers = buildUsedWireNumbers();

        var remaining = LED_STRIP.length - usedWireNumbers.length;
        
        $('.wires-remaining div').html(remaining);
    }

};

TABS.led_strip.cleanup = function (callback) {
    if (callback) callback();
};
