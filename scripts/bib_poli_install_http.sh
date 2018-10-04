#!/usr/bin/env bash

cp -r `rospack find bib_poli_package`/client ~/.pal/www/webapps/client/bibpoli

rostopic pub /web/go_to pal_web_msgs/WebGoTo 3 "/static/webapps/client/bibpoli/index.html"