'use strict';

$(document).ready(function () {
    function Dialog(identifier, content, handler) {
        var self = this;

        this.block = $('<div />').css({
            'position': 'fixed',
            'top': 0,
            'left': 0,
            'width': '100%',
            'height': '100%',
            'background-color': 'rgba(0, 0, 0, 0.25)',
            'z-index': 1000
        });

        $('body').append(this.block);

        this.element = $('<div />').prop('id', 'dialog').addClass(identifier).load(content, function () {
            // position the dialog
            self.element.css({
                'top': window.innerHeight / 3,
                'left': (window.innerWidth - self.element.width()) / 2
            });

            // display content
            self.element.fadeIn(100);

            if (handler) handler(self);
        });

        $('body').append(this.element);

        // handle window resize
        var resizeHandler = function () {
            self.element.css({
                'top': window.innerHeight / 3,
                'left': (window.innerWidth - self.element.width()) / 2
            });
        };

        $(window).on('resize', resizeHandler);


        // handle confirm/dismiss keys
        var keyDownHandler = function (e) {
            if (e.which == 13) {
                // Enter
                self.element.find('.yes').click();
            } else if (e.which == 27) {
                // ESC
                self.element.find('.no').click();
            }
        };

        $(document).on('keydown', keyDownHandler);

        // cleanup routine
        this.cleanup = function () {
            $(window).off('resize', resizeHandler);
            $(document).off('keydown', keyDownHandler);

            self.element.empty().remove();
            self.block.remove();
        };

        return this;
    }

    chrome.storage.sync.get('appReview', function (result) {
        if (typeof result.appReview !== 'undefined') {
            var data = result.appReview;

            if (data.launched < 10) {
                data.launched += 1;

                chrome.storage.sync.set({'appReview': data});
                return;
            }

            if ((data.firstStart + 604800000) < new Date().getTime()) {
                if ((data.refused == 0 || (data.refused + 604800000) < new Date().getTime()) && !data.reviewed) { // needs verifying
                    var dialog = new Dialog('review', './tabs/review.html', function () {
                        localize();

                        $('.initial', dialog.element).show();

                        var stage = 0;
                        $(dialog.element).on('click', '.yes, .no', function () {
                            if (!stage) {
                                $('p', dialog.element).hide();
                                if ($(this).hasClass('yes')) {
                                    $('.storeReview', dialog.element).show();
                                    stage = 1;
                                    googleAnalytics.sendEvent('Review', 'Likes App', true);
                                } else {
                                    $('.bugTicket', dialog.element).show();
                                    stage = 2
                                    googleAnalytics.sendEvent('Review', 'Likes App', false);
                                }
                                return false;
                            }

                            if (stage == 1) {
                                if ($(this).hasClass('yes')) {
                                    window.open('https://chrome.google.com/webstore/detail/baseflight-configurator/mppkgnedeapfejgfimkdoninnofofigk/reviews');
                                    data.reviewed = new Date().getTime();
                                    googleAnalytics.sendEvent('Review', 'Submits Review', true);
                                } else {
                                    data.refused = new Date().getTime();
                                    googleAnalytics.sendEvent('Review', 'Refused', true);
                                }
                            }

                            if (stage == 2) {
                                if ($(this).hasClass('yes')) {
                                    window.open('https://chrome.google.com/webstore/detail/baseflight-configurator/mppkgnedeapfejgfimkdoninnofofigk/support');
                                    data.refused = new Date().getTime();
                                    googleAnalytics.sendEvent('Review', 'Submits Bug Ticket', true);
                                } else {
                                    data.refused = new Date().getTime();
                                    googleAnalytics.sendEvent('Review', 'Refused', true);
                                }
                            }

                            chrome.storage.sync.set({'appReview': data});
                            dialog.cleanup();
                        });

                    });
                }
            }
        } else {
            // object not in storage, initial setup
            chrome.storage.sync.set({'appReview': {
                'firstStart':   new Date().getTime(),
                'launched':     1,
                'reviewed':     0,
                'refused':      0
            }});
        }
    });
});