"use strict";

var
    CHANNEL_MIN_VALUE = 1000,
    CHANNEL_MID_VALUE = 1500,
    CHANNEL_MAX_VALUE = 2000,
    
    // What's the index of each channel in the MSP channel list?
    channelMSPIndexes = {
        roll: 0,
        pitch: 1,
        yaw: 2,
        throttle: 3,
        aux1: 4,
        aux2: 5,
        aux3: 6,
        aux4: 7,
    },
    
    // Set reasonable initial stick positions (Mode 2)
    stickValues = {
        throttle: CHANNEL_MIN_VALUE,
        pitch: CHANNEL_MID_VALUE,
        roll: CHANNEL_MID_VALUE,
        yaw: CHANNEL_MID_VALUE,
        aux1: CHANNEL_MIN_VALUE,
        aux2: CHANNEL_MIN_VALUE,
        aux3: CHANNEL_MIN_VALUE,
        aux4: CHANNEL_MIN_VALUE
    },
    
    // First the vertical axis, then the horizontal:
    gimbals = [
        ["throttle", "yaw"],
        ["pitch", "roll"],
    ],
    
    gimbalElems,
    sliderElems,
    
    enableTX = false;

function transmitChannels() {
    var 
        channelValues = [0, 0, 0, 0, 0, 0, 0, 0];

    if (!enableTX) {
        return;
    }
    
    for (var stickName in stickValues) {
        channelValues[channelMSPIndexes[stickName]] = stickValues[stickName];
    }
    
    // Callback given to us by the window creator so we can have it send data over MSP for us:
    if (!window.setRawRx(channelValues)) {
        // MSP connection has gone away
        chrome.app.window.current().close();
    }
}

function stickPortionToChannelValue(portion) {
    portion = Math.min(Math.max(portion, 0.0), 1.0);
    
    return Math.round(portion * (CHANNEL_MAX_VALUE - CHANNEL_MIN_VALUE) + CHANNEL_MIN_VALUE);
}

function channelValueToStickPortion(channel) {
    return (channel - CHANNEL_MIN_VALUE) / (CHANNEL_MAX_VALUE - CHANNEL_MIN_VALUE);
}

function updateControlPositions() {
    for (var stickName in stickValues) {
        var
            stickValue = stickValues[stickName];
        
        // Look for the gimbal which corresponds to this stick name
        for (var gimbalIndex in gimbals) {
            var 
                gimbal = gimbals[gimbalIndex],
                gimbalElem = gimbalElems.get(gimbalIndex),
                gimbalSize = $(gimbalElem).width(),
                stickElem = $(".control-stick", gimbalElem);
            
            if (gimbal[0] == stickName) {
                stickElem.css('top', (1.0 - channelValueToStickPortion(stickValue)) * gimbalSize + "px");
                break;
            } else if (gimbal[1] == stickName) {
                stickElem.css('left', channelValueToStickPortion(stickValue) * gimbalSize + "px");
                break;
            }
        }
    }
}

function handleGimbalMouseDrag(e) {
    var 
        gimbal = $(gimbalElems.get(e.data.gimbalIndex)),
        gimbalOffset = gimbal.offset(),
        gimbalSize = gimbal.width();
    
    stickValues[gimbals[e.data.gimbalIndex][0]] = stickPortionToChannelValue(1.0 - (e.pageY - gimbalOffset.top) / gimbalSize);
    stickValues[gimbals[e.data.gimbalIndex][1]] = stickPortionToChannelValue((e.pageX - gimbalOffset.left) / gimbalSize);
    
    updateControlPositions();
}

function localizeAxisNames() {
    for (var gimbalIndex in gimbals) {
        var 
            gimbal = gimbalElems.get(gimbalIndex);
        
        $(".gimbal-label-vert", gimbal).text(chrome.i18n.getMessage("controlAxis" + gimbals[gimbalIndex][0]));
        $(".gimbal-label-horz", gimbal).text(chrome.i18n.getMessage("controlAxis" + gimbals[gimbalIndex][1]));
    }
    
    for (var sliderIndex = 0; sliderIndex < 4; sliderIndex++) {
        $(".slider-label", sliderElems.get(sliderIndex)).text(chrome.i18n.getMessage("controlAxisAux" + (sliderIndex + 1)));
    }
}

$(document).ready(function() {
    $(".button-enable").click(function() {
        var
            shrinkHeight = $(".warning").height();
        
        $(".warning").slideUp("short", function() {
            chrome.app.window.current().innerBounds.minHeight -= shrinkHeight;
            chrome.app.window.current().innerBounds.height -= shrinkHeight;
            chrome.app.window.current().innerBounds.maxHeight -= shrinkHeight;
        });
        
        enableTX = true;
    });
    
    gimbalElems = $(".control-gimbal");
    sliderElems = $(".control-slider");
    
    gimbalElems.each(function(gimbalIndex) {
        $(this).on('mousedown', {gimbalIndex: gimbalIndex}, function(e) {
            if (e.which == 1) { // Only move sticks on left mouse button
                handleGimbalMouseDrag(e);
                
                $(window).on('mousemove', {gimbalIndex: gimbalIndex}, handleGimbalMouseDrag);
            }
        });
    });
    
    $(".slider", sliderElems).each(function(sliderIndex) {
        var 
            initialValue = stickValues["aux" + (sliderIndex + 1)];
        
        $(this)
            .noUiSlider({
                start: initialValue,
                range: {
                    min: CHANNEL_MIN_VALUE,
                    max: CHANNEL_MAX_VALUE
                }
            }).on('slide change set', function(e, value) {
                value = Math.round(parseFloat(value));
                
                stickValues["aux" + (sliderIndex + 1)] = value;
                
                $(".tooltip", this).text(value);
            });
        
        $(this).append('<div class="tooltip"></div>');
        
        $(".tooltip", this).text(initialValue);
    });
    
    /* 
     * Mouseup handler needs to be bound to the window in order to receive mouseup if mouse leaves window.
     */
    $(window).mouseup(function(e) {
        $(this).off('mousemove', handleGimbalMouseDrag);
    });
    
    localizeAxisNames();
    
    updateControlPositions();
    
    setInterval(transmitChannels, 50);
});