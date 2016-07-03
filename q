[1mdiff --git a/_locales/en/messages.json b/_locales/en/messages.json[m
[1mindex 7ad20b6..dcebce2 100755[m
[1m--- a/_locales/en/messages.json[m
[1m+++ b/_locales/en/messages.json[m
[36m@@ -1691,8 +1691,27 @@[m
         "message": "Magnetometer (if supported)"[m
     },[m
       "PIDTip": {[m
[31m-        "message": "<strong><i>Derivative from Error</i></strong> provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[31m-    }  [m
[32m+[m[32m        "message": "<strong><i>Derivative from Error</i></strong> </br>Provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[32m+[m[32m    },[m
[32m+[m[32m      "PTip": {[m
[32m+[m[32m        "message": "<strong><i>Proportional</i></strong> </br>provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[32m+[m[32m    },[m
[32m+[m[32m      "ITip": {[m
[32m+[m[32m        "message": "<strong><i>Integral</i></strong> </br>provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[32m+[m[32m    },[m
[32m+[m[32m      "DTip": {[m
[32m+[m[32m        "message": "<strong><i>Derivative</i></strong> </br>provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[32m+[m[32m    },[m
[32m+[m[32m      "RateTip": {[m
[32m+[m[32m        "message": "<strong><i>Rate</i></strong> </br>provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[32m+[m[32m    },[m
[32m+[m[32m      "RcRateTip": {[m
[32m+[m[32m        "message": "<strong><i>Rc Rate</i></strong> </br>provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[32m+[m[32m    },[m
[32m+[m[32m      "ExpoTip": {[m
[32m+[m[32m        "message": "<strong><i>Rc Expo</i></strong> </br>provides more direct stick response and is mostly prefered for Racing.<br><strong><i>Derivative from Measurement</i></strong> provides smoother stick response what is more usefull for freestyling"[m
[32m+[m[32m    }[m[41m   [m
[32m+[m[41m    [m
     [m
 [m
     [m
[1mdiff --git a/tabs/pid_tuning.html b/tabs/pid_tuning.html[m
[1mindex fbae9bf..c025797 100755[m
[1m--- a/tabs/pid_tuning.html[m
[1m+++ b/tabs/pid_tuning.html[m
[36m@@ -44,7 +44,8 @@[m
                     <table class="pid_titlebar">[m
                         <tr>[m
                             <th class="name" i18n="pidTuningName"></th>[m
[31m-                            <th class="proportional" i18n="pidTuningProportional"></th>[m
[32m+[m[32m                            <th class="proportional helpicon cf_tip" i18n="pidTuningProportional">[m
[32m+[m[32m                            <div class="helpicon cf_tip" i18n_title="PTip"></div></th>[m
                             <th class="integral" i18n="pidTuningIntegral"></th>[m
                             <th class="derivative" i18n="pidTuningDerivative"></th>[m
                             <th class="rc_rate" i18n="pidTuningRcRate"></th>[m
