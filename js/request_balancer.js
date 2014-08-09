'use strict';

function request_delay_balancer(refresh_period) {
    this.balance_to = refresh_period;
    this.request_t = 0;
    this.finished_t = 0;
}

request_delay_balancer.prototype.requested = function () {
    this.request_t = millitime();
};

request_delay_balancer.prototype.finished = function () {
    this.finished_t = millitime();
};

request_delay_balancer.prototype.estimate = function () {
    var estimate = this.balance_to - (this.finished_t - this.request_t);
    return (estimate > 0) ? estimate : 0;
};