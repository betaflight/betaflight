'use strict';

var minRc = 1000;
var midRc = 1500;
var maxRc = 2000;

var RateCurve = function (useLegacyCurve) {
    this.useLegacyCurve = useLegacyCurve;

    function constrain(value, min, max) {
        return Math.max(min, Math.min(value, max));
    }

    function rcCommand(rcData, rcRate, rcExpo) {
        var tmp = Math.min(Math.abs(rcData - midRc), 500) / 100;

	var result = ((2500 + rcExpo * (tmp * tmp - 25)) * tmp * rcRate / 2500).toFixed(0);
	if (rcData < midRc) {
            result = -result;
        }

        return result;
    }

    this.rcCommandRawToDegreesPerSecond = function (rcData, rate, rcRate, rcExpo, superExpoActive) {
        var inputValue = rcCommand(rcData, rcRate, rcExpo);

        var angleRate;
        if (superExpoActive) {
            var rcFactor = Math.abs(inputValue) / (500 * rcRate / 100);
            rcFactor = 1 / constrain(1 - rcFactor * rate / 100, 0.01, 1);

            angleRate = rcFactor * 27 * inputValue / 16;
        } else {
            angleRate = (rate + 27) * inputValue / 16;
        }

        angleRate = constrain(angleRate, -8190, 8190); // Rate limit protection
        return angleRate >> 2; // the shift by 2 is to counterbalance the divide by 4 that occurs on the gyro to calculate the error
    };

    this.drawRateCurve = function (rate, rcRate, rcExpo, superExpoActive, maxAngularVel, context, width, height) {
        rate = rate * 100;
	rcRate = rcRate * 100;
	rcExpo = rcExpo * 100;

        var canvasHeightScale = height / (2 * maxAngularVel);

        var stepWidth = context.lineWidth;

        context.save();
        context.translate(width / 2, height / 2);

        context.beginPath();
        var rcData = minRc;
        context.moveTo(-500, -canvasHeightScale * this.rcCommandRawToDegreesPerSecond(rcData, rate, rcRate, rcExpo, superExpoActive));
        rcData = rcData + stepWidth;
        while (rcData <= maxRc) {
            context.lineTo(rcData - midRc, -canvasHeightScale * this.rcCommandRawToDegreesPerSecond(rcData, rate, rcRate, rcExpo, superExpoActive));

            rcData = rcData + stepWidth;
        }
        context.stroke();

        context.restore();
    }

    this.drawLegacyRateCurve = function (rate, rcRate, rcExpo, context, width, height) {
        // math magic by englishman
        var rateY = height * rcRate;
        rateY = rateY + (1 / (1 - ((rateY / height) * rate)))

        // draw
        context.beginPath();
        context.moveTo(0, height);
        context.quadraticCurveTo(width * 11 / 20, height - ((rateY / 2) * (1 - rcExpo)), width, height - rateY);
        context.stroke();
    }
}

RateCurve.prototype.getMaxAngularVel = function (rate, rcRate, rcExpo, superExpoActive) {
    var maxAngularVel;
    if (rate !== undefined && rcRate !== undefined && rcExpo !== undefined
        && !this.useLegacyCurve) {
        rate = rate * 100;
	rcRate = rcRate * 100;
	rcExpo = rcExpo * 100;

        maxAngularVel = this.rcCommandRawToDegreesPerSecond(maxRc, rate, rcRate, rcExpo, superExpoActive);
    }

    return maxAngularVel;
}

RateCurve.prototype.draw = function (rate, rcRate, rcExpo, superExpoActive, maxAngularVel, context) {
    if (rate !== undefined && rcRate !== undefined && rcExpo !== undefined) {
        var height = context.canvas.height;
        var width = context.canvas.width;

        if (this.useLegacyCurve) {
            this.drawLegacyRateCurve(rate, rcRate, rcExpo, context, width, height);
        } else {
            this.drawRateCurve(rate, rcRate, rcExpo, superExpoActive, maxAngularVel, context, width, height);
        }
    }
}
