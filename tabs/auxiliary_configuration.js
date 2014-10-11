'use strict';

// TODO: rework box_highlight & update_ui to accept flexible amount of aux channels
TABS.auxiliary_configuration = {};
TABS.auxiliary_configuration.initialize = function (callback) {
    GUI.active_tab_ref = this;
    GUI.active_tab = 'auxiliary_configuration';
    googleAnalytics.sendAppView('Auxiliary Configuration');
    
    function get_box_data() {
        MSP.send_message(MSP_codes.MSP_BOX, false, false, get_box_ids);
    }

    function get_box_ids() {
        MSP.send_message(MSP_codes.MSP_BOXIDS, false, false, get_rc_data);
    }

    function get_rc_data() {
        MSP.send_message(MSP_codes.MSP_RC, false, false, load_html);
    }

    function load_html() {
        $('#content').load("./tabs/auxiliary_configuration.html", process_html);
    }

    MSP.send_message(MSP_codes.MSP_BOXNAMES, false, false, get_box_data);

    function createMode(modeIndex) {
        var modeTemplate = $('#templates .mode');
        var newMode = modeTemplate.clone();
        
        var modeName = AUX_CONFIG[modeIndex];
        $(newMode).attr('id', 'mode-' + modeIndex);
        $(newMode).find('.name').text(modeName);
        
        $(newMode).data('index', modeIndex);
        
        $(newMode).find('.name').data('modeElement', newMode);
        $(newMode).find('a.addRange').data('modeElement', newMode);

        return newMode; 
    }
    
    function configureRangeTemplate(auxChannelCount) {

        var rangeTemplate = $('#templates .range');
        
        var channelList = $(rangeTemplate).find('.channel');
        var channelOptionTemplate = $(channelList).find('option');
        channelOptionTemplate.remove();
        for (var channelIndex = 0; channelIndex < auxChannelCount; channelIndex++) {
            var channelOption = channelOptionTemplate.clone();
            channelOption.text('AUX ' + (channelIndex + 1));
            channelOption.val(channelIndex);
            channelList.append(channelOption);
        }
        channelList.select(0);
    }
    
    function addRangeToMode(modeElement, auxChannelIndex) {

        var modeIndex = $(modeElement).data('index');

        var channel_range = {
                'min': [  900 ],
                'max': [ 2100 ]
            };

        var rangeIndex = $(modeElement).find('.range').length;
        
        var range = $('#templates .range').clone();
        range.attr('id', 'mode-' + modeIndex + '-range-' + rangeIndex);
        modeElement.find('.ranges').append(range);
        
        $(range).find('.channel-slider').noUiSlider({
            start: [ 1400, 1600 ],
            behaviour: 'snap-drag',
            step: 25,
            connect: true,
            range: channel_range,
            format: wNumb({
                decimals: 0,
            })
        });

        var elementName =  '#mode-' + modeIndex + '-range-' + rangeIndex;
        $(elementName + ' .channel-slider').Link('lower').to($(elementName + ' .lowerLimitValue'));
        $(elementName + ' .channel-slider').Link('upper').to($(elementName + ' .upperLimitValue'));

        $(range).find(".pips-channel-range").noUiSlider_pips({
            mode: 'values',
            values: [900, 1000, 1200, 1400, 1500, 1600, 1800, 2000, 2100],
            density: 4,
            stepped: true
        });
        
        $(range).find('.deleteRange').data('rangeElement', range);

        $(range).find('a.deleteRange').click(function () {
            var rangeElement = $(this).data('rangeElement');
            rangeElement.remove();
        });
        
        $(range).find('.channel').val(auxChannelIndex);

    }
    
    function process_html() {
        
        $('.boxes').hide();

        var auxChannelCount = RC.active_channels - 4;

        configureRangeTemplate(auxChannelCount);

        var modeTableBodyElement = $('.tab-auxiliary_configuration .modes tbody') 
        for (var modeIndex = 0; modeIndex < AUX_CONFIG.length; modeIndex++) {
            
            var newMode = createMode(modeIndex);
            modeTableBodyElement.append(newMode);

            if (AUX_CONFIG_values[modeIndex] > 0) {
                $('.mode .name').eq(modeIndex).data('modeElement').addClass('off');
            }
        }

        function findFirstUnusedChannel(modeElement) {
            var auxChannelIndexCandidates = [];
            for (var auxChannelIndex = 0; auxChannelIndex < auxChannelCount; auxChannelIndex++) {
                auxChannelIndexCandidates.push(auxChannelIndex);
            }
            
            $(modeElement).find('.channel').each( function() {
                var valueToRemove = $(this).val();
                auxChannelIndexCandidates = auxChannelIndexCandidates.filter(function(item) {
                    return item != valueToRemove;
                });
            });
            
            return auxChannelIndexCandidates[0];
        }
        
        $('a.addRange').click(function () {
            var modeElement = $(this).data('modeElement');
            
            var firstUnusedChannel = findFirstUnusedChannel(modeElement);
            
            addRangeToMode(modeElement, firstUnusedChannel);
        });
        
        // generate heads according to RC count
        var table_head = $('table.boxes .heads');
        var main_head = $('table.boxes .main');
        for (var i = 0; i < (RC.active_channels - 4); i++) {
            table_head.append('<th colspan="3">AUX ' + (i + 1) + '</th>');

            // 3 columns per aux channel (this might be requested to change to 6 in the future, so watch out)
            main_head.append('\
                <th i18n="auxiliaryLow"></th>\
                <th i18n="auxiliaryMed"></th>\
                <th i18n="auxiliaryHigh"></th>\
            ');
        }

        // translate to user-selected language
        localize();

        // generate table from the supplied AUX names and AUX data
        for (var i = 0; i < AUX_CONFIG.length; i++) {
            var line = '<tr class="switches">';
            line += '<td class="name">' + AUX_CONFIG[i] + '</td>';


            var switches = '';
            var auxChannelCount = RC.active_channels - 4;
            
            var bitIndex = 0;
            var chunks = 1;
            if (bit_check(CONFIG.capability, 5) && (auxChannelCount) > 4) {
                chunks = 2;
            }
            
            var channelsRemaining = auxChannelCount;
            var channelsPerChunk = 4;
            for (var chunk = 0; chunk < chunks; chunk++) {
                for (var chunkChannel = 0; chunkChannel < channelsPerChunk && channelsRemaining; chunkChannel++, channelsRemaining--) {
                    for (var j = 0; j < 3; j++) {
                        if (bit_check(AUX_CONFIG_values[i], bitIndex++)) {
                            switches += '<td><input type="checkbox" checked="checked" /></td>';
                        } else {
                            switches += '<td><input type="checkbox" /></td>';
                        }
                    }
                }
                bitIndex += 16 - (4 * 3);
            }
            
            line += switches + '</tr>';

            $('.boxes > tbody:last').append(line);
        }

        // UI Hooks
        $('a.save').click(function () {

            // TODO update internal data structures based on current UI elements
            
            // TODO send data to FC

            /* snippets of old code that might be useful...
            var AUX_val_buffer_out = [];
            AUX_val_buffer_out.push(lowByte(0xFFFF));
            AUX_val_buffer_out.push(highByte(0xFFFF));

            MSP.send_message(MSP_codes.MSP_SET_BOX, AUX_val_buffer_out, false, save_to_eeprom);

            function save_to_eeprom() {
                MSP.send_message(MSP_codes.MSP_EEPROM_WRITE, false, false, function () {
                    GUI.log(chrome.i18n.getMessage('auxiliaryEepromSaved'));
                });
            }
            */
        });

       
        function box_highlight(auxChannelIndex, channelPosition) {
            if (channelPosition < 900) {
                channelPosition = 900;
            } else if (channelPosition > 2100) {
                channelPosition = 2100;
            }
            var percentage = (channelPosition - 900) / (2100-900) * 100;
            
            $('.modes .ranges .range').each( function () {
                var auxChannelCandidateIndex = $(this).find('.channel').val();
                if (auxChannelCandidateIndex != auxChannelIndex) {
                    return;
                }
                
                $(this).find('.marker').css('left', percentage + '%');
            });
            
        }

        // data pulling functions used inside interval timer
        function get_rc_data() {
            MSP.send_message(MSP_codes.MSP_RC, false, false, update_ui);
        }

        function update_ui() {
            for (var i = 0; i < AUX_CONFIG.length; i++) {
                if (AUX_CONFIG_values[i] == 0) {
                    continue;
                }
                
                if (bit_check(CONFIG.mode, i)) {
                    $('.mode .name').eq(i).data('modeElement').addClass('on').removeClass('off');
                } else {
                    $('.mode .name').eq(i).data('modeElement').removeClass('on').addClass('off');
                }
            }

            var auxChannelCount = RC.active_channels - 4;

            for (var i = 0; i < (auxChannelCount); i++) {
                box_highlight(i, RC.channels[i + 4]);
            }           
        }

        // update ui instantly on first load
        update_ui();

        // enable data pulling
        GUI.interval_add('aux_data_pull', get_rc_data, 50);

        // status data pulled via separate timer with static speed
        GUI.interval_add('status_pull', function () {
            MSP.send_message(MSP_codes.MSP_STATUS);
        }, 250, true);

        if (callback) callback();
    }
};

TABS.auxiliary_configuration.cleanup = function (callback) {
    if (callback) callback();
};