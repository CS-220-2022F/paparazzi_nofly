grep -c --color "$1" $(grep -lr "$1" $PAPARAZZI_HOME)
