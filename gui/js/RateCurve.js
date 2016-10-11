'use strict';

var minRc = 1000;
var midRc = 1500;
var maxRc = 2000;

var RateCurve = function (useLegacyCurve) {
    this.useLegacyCurve = useLegacyCurve;
    this.maxAngularVel = null;

    this.constrain = function (value, min, max) {
        return Math.max(min, Math.min(value, max));
    };

    this.rcCommand = function (rcData, rcRate, deadband) {
        var tmp = Math.min(Math.max(Math.abs(rcData - midRc) - deadband, 0), 500);

        var result = tmp * rcRate;

        if (rcData < midRc) {
            result = -result;
        }

        return result;
    };

    this.drawRateCurve = function (rate, rcRate, rcExpo, superExpoActive, deadband, maxAngularVel, context, width, height) {
        var canvasHeightScale = height / (2 * maxAngularVel);

        var stepWidth = context.lineWidth;

        context.save();
        context.translate(width / 2, height / 2);

        context.beginPath();
        var rcData = minRc;
        context.moveTo(-500, -canvasHeightScale * this.rcCommandRawToDegreesPerSecond(rcData, rate, rcRate, rcExpo, superExpoActive, deadband));
        rcData = rcData + stepWidth;
        while (rcData <= maxRc) {
            context.lineTo(rcData - midRc, -canvasHeightScale * this.rcCommandRawToDegreesPerSecond(rcData, rate, rcRate, rcExpo, superExpoActive, deadband));

            rcData = rcData + stepWidth;
        }
        context.stroke();

        context.restore();
    };

    this.drawLegacyRateCurve = function (rate, rcRate, rcExpo, context, width, height) {
        // math magic by englishman
        var rateY = height * rcRate;
        rateY = rateY + (1 / (1 - ((rateY / height) * rate)));

        // draw
        context.beginPath();
        context.moveTo(0, height);
        context.quadraticCurveTo(width * 11 / 20, height - ((rateY / 2) * (1 - rcExpo)), width, height - rateY);
        context.stroke();
    }

    this.drawStickPosition = function (rcData, rate, rcRate, rcExpo, superExpoActive, deadband, maxAngularVel, context, stickColor) {

        const DEFAULT_SIZE = 60; // canvas units, relative size of the stick indicator (larger value is smaller indicator)
        const rateScaling  = (context.canvas.height / 2) / maxAngularVel;

        var currentValue = this.rcCommandRawToDegreesPerSecond(rcData, rate, rcRate, rcExpo, superExpoActive, deadband);

        if(rcData!=undefined) {
            context.save();
            context.fillStyle = stickColor || '#000000';

            context.translate(context.canvas.width/2, context.canvas.height/2);
            context.beginPath();
            context.arc(rcData-1500, -rateScaling * currentValue, context.canvas.height / DEFAULT_SIZE, 0, 2 * Math.PI);
            context.fill();
            context.restore();
        }
        return (Math.abs(currentValue)<0.5)?0:currentValue.toFixed(0); // The calculated value in deg/s is returned from the function call for further processing.
    }

};

RateCurve.prototype.rcCommandRawToDegreesPerSecond = function (rcData, rate, rcRate, rcExpo, superExpoActive, deadband) {
    var angleRate;
    if (rate !== undefined && rcRate !== undefined && rcExpo !== undefined) {
        if (rcRate > 2) {
            rcRate = rcRate + (rcRate - 2) * 14.54;
        }

        var inputValue = this.rcCommand(rcData, rcRate, deadband);
        var maxRc = 500 * rcRate;
        
        var expoPower;
        var rcRateConstant;
        if (semver.gte(CONFIG.flightControllerVersion, "3.0.0")) {
            expoPower = 3;
            rcRateConstant = 200;
        } else {
            expoPower = 2;
            rcRateConstant = 205.85;
        }

        if (rcExpo > 0) {
            var absRc = Math.abs(inputValue) / maxRc;
            inputValue =  inputValue * Math.pow(absRc, expoPower) * rcExpo + inputValue * (1-rcExpo);
        }

        var rcInput = inputValue / maxRc;

        if (superExpoActive) {
            var rcFactor = 1 / this.constrain(1 - Math.abs(rcInput) * rate, 0.01, 1);
            angleRate = rcRateConstant * rcRate * rcInput; // 200 should be variable checked on version (older versions it's 205,9)
            angleRate = angleRate * rcFactor;
        } else {
            angleRate = (((rate * 100) + 27) * inputValue / 16) / 4.1; // Only applies to old versions ?
        }

        angleRate = this.constrain(angleRate, -1998, 1998); // Rate limit protection
    }

    return angleRate;
};

RateCurve.prototype.getMaxAngularVel = function (rate, rcRate, rcExpo, superExpoActive, deadband) {
    var maxAngularVel;
    if (!this.useLegacyCurve) {
        maxAngularVel = this.rcCommandRawToDegreesPerSecond(maxRc, rate, rcRate, rcExpo, superExpoActive, deadband);
    }

    return maxAngularVel;
};

RateCurve.prototype.setMaxAngularVel = function (value) {
    this.maxAngularVel = Math.ceil(value/200) * 200;
    return this.maxAngularVel;

};

RateCurve.prototype.draw = function (rate, rcRate, rcExpo, superExpoActive, deadband, maxAngularVel, context) {
    if (rate !== undefined && rcRate !== undefined && rcExpo !== undefined) {
        var height = context.canvas.height;
        var width = context.canvas.width;

        if (this.useLegacyCurve) {
            this.drawLegacyRateCurve(rate, rcRate, rcExpo, context, width, height);
        } else {
            this.drawRateCurve(rate, rcRate, rcExpo, superExpoActive, deadband, maxAngularVel, context, width, height);
        }
    }
};
